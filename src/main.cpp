/**
 * MEGADOOR - It's a thing.
 *
 * This firmware is intended for an Atmega328P;
 * it will read NFC UIDs from an i2c connection
 * and unlock a door if the card is valid.
 */

/* TODO
  In no particular order:
  - Order the TODO list
  - Implement LPM
  - Implement USB Host (to make USB keys work)
  - Implement BIGSIGN and/or RQ Welcome message
 */

/////////////
// HEADERS //
/////////////

#include "main.h"

//////////
// PINS //
//////////

// Required for I2C
#define PIN_DOOR_UNLOCK (9) // Trips the transistor/relay switch
#define PIN_TROUBLE (13) // Wired to LED
/* !I2C!
  There's an implied SDA&SCL pin here.
  Both I2C libraries uses the ATmega's
  default I2C pins (27&28 on the DIP) to reach
  the NFC board. Make sure these pins are wired.
  Both of these pins also require a pull-up resistor
  because neither the ATmega nor PN532 boards have
  the hardware.
  Put a 1.5K resistor to VCC on each line.
 */

/////////////
// GLOBALS //
/////////////

#define debug 1
#define use_nfc 1
#define MAIN_DEBUG 1

// Set version MAJ.MIN-REV
#define VERSION "0.2.0-0"
#define BACKSPACE "\b \b"

// Serial buffer
#define BUF_LEN 48
char in_char = '\0';
char in_buffer[BUF_LEN+1];
unsigned short buffer_index = 0;
short int input_state = 0; // Current state for reading serial input.
/* Serial State Machine
 * 0: Out of state, discard. ':' -> go to 1.
 * 1: Record command. '\n' -> go to 2.
 * 2: Process command and go to 0.
 */

// Stats
// uint32_t max: 65535
unsigned long int cycle_count = 0; // Number of (15s) polling cycles
uint32_t valid_count = 0; // Number of verified UIDs processed
uint32_t invalid_count = 0; // Number of incorrect UIDs processed

// State definitions
#define MAIN_STATE_INIT 		(0x00)
#define MAIN_STATE_AUTOPOLL 	(0x01)
#define MAIN_STATE_LPM 			(0x02)
#define MAIN_STATE_CHECKCARD 	(0x03)
#define MAIN_STATE_READKEY 		(0x04)

static uint8_t state; // The global state for the main application
uint8_t nbTg; // Number of targets
//uint8_t type1 = response[3];
uint8_t nfcid_len;
uint8_t nfcid[7];
uint8_t pageNum;

////////////////
// PROTOTYPES //
////////////////

void watchdogSetup();
void watchdogTrip();
void door_unlock();
void door_lock();
void buffer_reset();
void buffer_process(bool loud = false);

#define reset()        		\
do                          \
{                           \
    wdt_enable(WDTO_15MS);  \
    for(;;)                 \
    {                       \
    }                       \
} while(0)

#define bootloader_asm()	\
__asm__ __volatile__ ("jmp 0x7e00"::);

// http://www.nongnu.org/avr-libc/user-manual/group__avr__watchdog.html
uint8_t mcusr_mirror __attribute__ ((section (".noinit")));
void get_mcusr(void) __attribute__((naked)) \
__attribute__((used)) __attribute__((section(".init3")));

void get_mcusr(void)
{
    MCUSR = 0;
    wdt_disable();
    __asm__ __volatile__ ("sts %0, r2\n" : "=m" (mcusr_mirror) :);
}

uint8_t do_sleep(uint8_t mode)
{
    // Debugging output
    if (MAIN_DEBUG) printf("Sleep, 0x%02X... ", mode);

    if (PIND & (1<<PIN0))
    {
        if (MAIN_DEBUG) printf("UART active, skipping.\n");
        return(0);
//		printf("Using IDLE mode... ");
//		set_sleep_mode(SLEEP_MODE_IDLE);
    }
    else
    {
        set_sleep_mode(mode);
    }

    // Atomic sleep code
    // http://www.nongnu.org/avr-libc/user-manual/group__avr__sleep.html
    cli();
    if ((PIND & (1 << PIN3)))
    {
        sleep_enable();
        sleep_bod_disable();
        sei();
        sleep_cpu();
        sleep_disable();
    }
    sei();

    // Debugging output
    if (MAIN_DEBUG) printf(" Done.\n");

    return(0);
}

uint8_t cb_firmwareVersion(uint8_t * response, uint8_t len)
{
    if (len != 6)
    {
        return(1);
    }
    printf("Firmware: %d.%d\n", response[3], response[4]);
    return(0);
}

uint8_t cb_generalStatus(uint8_t * response, uint8_t len)
{
    if (MAIN_DEBUG)
    {
        printf("Gen Status:\n");
        printf("\tErr code: 0x%02X\n", response[2]);
        printf("\tField: 0x%02X\n", response[3]);
        printf("\t# tags: 0x%02X\n", response[4]);
    }

    if (response[2] != 0)
    {
        printf("Error, restarting.\n");
        _delay_ms(250);
        pn532_getGeneralStatus(cb_generalStatus);
        pn532_blockForCallback();
    }
    return(0);
}

uint8_t cb_SAMConfiguration(uint8_t * response, uint8_t len)
{
    if (MAIN_DEBUG) printf("SAM config done.\n");
    return(0);
}

uint8_t cb_powerDown(uint8_t * response, uint8_t len)
{
    if (MAIN_DEBUG) printf("Power down. status: 0x%02X\n", response[2]);
    return(0);
}

uint8_t cb_inAutoPoll(uint8_t * response, uint8_t len)
{
    if (len != 17 && len != 14)
    {
//		led_set(10, 10, 0);
        nbTg = 0;
		printf("Len %d is not 17 or 14, no card available.\n", len);
//		_delay_ms(2);
//		led_set(0, 0, 0);
        return(0); // Not really an error.
    }
    nbTg = response[2];
    //type1 = response[3];
    nfcid_len = response[9];

    if (MAIN_DEBUG)
    {
        printf("%d cards available.\nUID: ", nbTg);
        for (uint8_t i = 0; i < nfcid_len; i++)
        {
            nfcid[i] = response[i+10];
            printf("0x%02X ", nfcid[i]);
        }
        printf("\n");
    }

    uint8_t text[20];
    uint16_t key_index;

    SHA1Context sha;
    SHA1Reset(&sha);

    for (uint8_t ind = 0; ind < nfcid_len; ind++)
    {
        uint8_t c[4];
        sprintf((char*)c, "%02x", nfcid[ind]);
        SHA1Input(&sha, c, 2);
    }
    printf("\n");

    SHA1Result(&sha, text);

    printf("sha1: ");
    for (uint8_t index = 0; index < 20; index++)
    {
        printf("%02x", text[index]);
    }
	printf("\n");

    uint8_t match_flag = 1;

    key_index = key_max_index();

    if (key_check(text, &key_index) == 0)
	{
		printf("Door unlock\n");
		door_unlock();
		valid_count++;
		return(0);
	}
	else
	{
		invalid_count++;
	}

//	led_set(0, 0, 0);
    return(0);
}

uint8_t blockForCallback()
{
    uint8_t retval;
    retval = 0;
    while (PN532_STATE_RESTING != pn532_getState())
    {
//        proc_serial();
        if ((retval = pn532_poll()))
            break;
    }
    return(retval);
}

uint8_t procForCallback()
{
    uint8_t retval;
//    printf("\tProcessing pn532.\n");
	pn532_poll();
    retval = pn532_getState();
    return(retval);
}

uint8_t proc_pn532()
{
    // Variables
    uint8_t retval;
    uint8_t procStatus;

    // Instantiation
    retval = 0;
    procStatus = procForCallback();

//    printf("procstatus: %d, state: %d\n", procStatus, state);

    if (procStatus && (state == MAIN_STATE_AUTOPOLL))
	{
//		printf("Nothing to do in proc_pn532.\n");
		return(retval);
	}
	else if (state == MAIN_STATE_AUTOPOLL)
	{
//		printf("Done processing for callback, moving along.\n");
		state = MAIN_STATE_INIT;
	}

    if((retval = pn532_inAutoPoll(NFC_POLLING_NUM, NFC_POLLING_LEN, 0x10, cb_inAutoPoll)))
    {
        printf("pn532_inAutoPoll failed 0x%02X.\n", retval);
        pn532_recover();
		state = MAIN_STATE_INIT;
        return(1);
    }
    state = MAIN_STATE_AUTOPOLL;
    printf("Waiting for card.\n");
    cycle_count++;

    return retval;
}

void watchdogSetup(void)
{
    cli(); // disable all interrupts
    wdt_reset(); // reset the WDT timer
    /*
    WDTCSR configuration:
    WDIE = 1: Interrupt Enable
    WDE = 1 :Reset Enable
    WDP3 = 0 :For 2000ms Time-out
    WDP2 = 1 :For 2000ms Time-out
    WDP1 = 1 :For 2000ms Time-out
    WDP0 = 0 :For 2000ms Time-out
    */
    // Enter Watchdog Configuration mode:
    WDTCSR |= (1<<WDCE) | (1<<WDE);
    // Set Watchdog settings:
    WDTCSR = (1<<WDIE) | (1<<WDE) | (0<<WDP3) | (1<<WDP2) | (1<<WDP1) | (0<<WDP0);
    sei();
}

void watchdogTrip(void)
{
    // Enter Watchdog Configuration mode:
    //WDTCSR |= (1<<WDCE) | (1<<WDE);
    // Set Watchdog settings:
    //WDTCSR |= (0<<WDIF);
    _delay_ms(10000);
}

void door_unlock ()
{
    wdt_reset();

	PORTB &= ~(2 << PB1);
	_delay_ms(250);
	PORTB |= (2 << PB1);

    for (uint8_t i = 0; i < 6; i++)
    {
        wdt_reset();
        _delay_ms(500);
    }
}

void door_lock ()
{
    // Ha-HA!
}

void buffer_reset ()
{
    for (buffer_index = 0; buffer_index < BUF_LEN; ++buffer_index)
    {
        in_buffer[buffer_index] = '\0';
    }
    buffer_index = 0;
}

void buffer_process(bool loud /*= false*/)
{
	uint8_t cmd_len = strlen(in_buffer);
	if (MAIN_DEBUG) loud = 1;

    if (loud)
    {
        printf("Serial command: [%s], command length: [%d]\n", in_buffer, cmd_len);
    }

	// Compatibility commands for working with ACE utilities
    if (!strncmp(in_buffer, "BEEFB0BOOT", 10))
	{
		printf("[RB!]\n"); // Restart Bootloader
		pn532_cancellCmd();
		bootloader_asm();
	}
	else if (!strncmp(in_buffer, "BEEFB0STRT", 10))
	{
		printf("[RA!]\n"); // Restart Application
		pn532_cancellCmd();
		reset();
	}

	// Commands for working with megadoor
    if (in_buffer[0] == 'd') // Door
    {
        if (loud) printf("Setting door: ");
        if (in_buffer[1] == 'u') // Door Unlock
        {
            if (loud) printf("UNLOCKED.\n");
            //watchdogTrip();
            door_unlock();
        } // end Door Unlock
        else if (in_buffer[1] == 'l') // Door Lock
        {
            if (loud) printf("LOCKED.\n");
            door_lock();
        } // end Door Lock
    } // end Door

    else if (in_buffer[0] == 'g') // Get
	{
		if (in_buffer[1] == 'c') // Get cycles
		{
			if (loud)
			{
				uint16_t nfc_timeout = (NFC_POLLING_TIMEOUT/1000); // In seconds.
				printf("Nfc timeout: %d\n", nfc_timeout);
//				uint16_t uptime =
				printf("I have survived %u cycles. Approximately %u days.\n", cycle_count, ((cycle_count*1.5)/8640));
				printf("I have verified %u valid NFC tags, and rejected %u invalid NFC tags.", valid_count, invalid_count);
			}
			else printf("%u\n", cycle_count);
		}
		else if (in_buffer[1] == 'k') // Get key (Same as Key get)
		{
			in_buffer[0] = 'k';
			in_buffer[1] = 'g';
			buffer_process(loud);
		}
	} // end Get

	// All keys are sent as either key indexes or sha1 sums of the key UID.
	else if (in_buffer[0] == 'k') // Key
	{
		if (loud) printf("Key operation: ");
		if (in_buffer[1] == 'g') // Key get
		{
			uint8_t key_index = atoi(&in_buffer[2]);
			if (loud) printf("Key index: %d\n", key_index);
			uint8_t tmp_key[KEY_MAX_LEN];
			key_get(key_index, tmp_key);
			for (uint8_t index = 0; index < KEY_MAX_LEN; index++)
			{
				if (loud) printf("0x%02X ", tmp_key[index]);
				else printf("%02X", tmp_key[index]);
			}
			printf("\n");
		} // End key get
		else if (in_buffer[1] == 'a') // Key add (sha1) [kaXXXXXXXXXXXXXXXXXXXX]
		{
			uint8_t tmp_key[KEY_MAX_LEN] = {0};
			uint8_t index = 2;
			uint8_t tmp_addr = 0;
			while (index < BUF_LEN && tmp_addr < KEY_MAX_LEN && in_buffer[index] != ']')
			{
//				tmp_key[index-2] =
				char holder[3];
				holder[0] = in_buffer[index];
				holder[1] = in_buffer[index+1];
				holder[2] = '\0';
				tmp_key[tmp_addr] = strtol(holder, NULL, 16);
//				printf("Adding: 0x%02X\n", tmp_key[tmp_addr]);
				tmp_addr++;
				index += 2;
			}
			if (loud)
			{
				printf("Adding key: \n");
				for (uint8_t index = 0; index < KEY_MAX_LEN; index++)
				{
					printf("0x%02X ", tmp_key[index]);
				}
				printf("\n");
			}
			if (key_add(tmp_key))
			{
				printf("Key add failed.\n");
			}
			else
			{
				if (loud) printf("Key add success.\n");
			}
		} // End key add
		else if (in_buffer[1] == 'd') // Key delete
		{
			if (loud) printf("delete ");
			if (in_buffer[2] == 'a') // all
			{
				if (loud) printf("all\n");
				for (int i = 0; i < key_max_index(); i++)
				{
//					printf("removing key %d ", i);
					key_remove(i);
//					printf("done.\n");
				}
				if (loud) printf("Keys removed.\n");
			}
			else
			{
				if (loud) printf("\n");
				uint8_t key_index = atoi(&in_buffer[2]);
				if (loud) printf("Key index: %d\n", key_index);
				uint8_t tmp_key[KEY_MAX_LEN];
				key_get(key_index, tmp_key);
				if (loud)
				{
					printf("Deleting key: ");
					for (uint8_t index = 0; index < KEY_MAX_LEN; index++)
					{
						printf("0x%02X ", tmp_key[index]);
					}
					printf("\n");
				}
				if (key_remove(key_index))
				{
					printf("Key remove failed.\n");
				}
				else
				{
					if (loud) printf("Key remove success.\n");
				}
			}
		} // End key delete
		else if (in_buffer[1] == 'c') // Key check
		{
			// Not implemented
		} // End key check
	}

    buffer_reset();
} // end buffer_process()

/* Init
 * This function is kept over from the Arduino library for no reason.
 * I have arbitrarily decided that it will be used to initialize
 * low level hardware.
 */
void init ()
{
	// Pin setup
	DDRB = (2 << PB1); // Set as output
	DDRD = (2 << PD6); // Set as output

	PORTB |= (2 << PB1);
}

/**
 * setup() - a poem
 * We do what we must \
 * because they make us.
 */
void setup()
{
//	watchdogSetup();

    // Printf setup
    usart_init();
    init_printf(NULL, usart_putchar);
    printf("usart init.\n");

    // Debugging header output
    printf("Flash key storage: \t%u keys.\n", FLASH_NUM_KEYS);
    printf("EEPROM key storage: \t%d keys.\n", EEPROM_NUM_KEYS);
    printf("Key maximum length: \t%d bytes.\n", KEY_MAX_LEN);
    printf("Key maximum index: \t%u.\n", key_max_index());
    printf("EEPROM start: \t0x%02X\n", EEPROM_ADDR_START);
    printf("EEPROM end: \t0x%02X\n", EEPROM_ADDR_END);
    printf("Flash start: \t0x%02X\n", FLASH_ADDR_START);
    printf("Flash end: \t0x%02X\n", FLASH_ADDR_END);

//    for (uint16_t index = 0; index <= key_max_index(); index++)
//	{
//		uint8_t holder[20];
//		key_get(index, holder);
//		printf("Key %d: ", index);
//		for (uint8_t byte_ = 0; byte_ < 20; byte_++)
//		{
//			printf("0x%02X, ", holder[byte_]);
//		}
//		printf("\n");
//	}

    if (debug)
    {
        printf("megadoor version %s\n", VERSION);
        printf("I have %d bytes EEPROM.\n", E2END);
    }

//    eeprom_format();

	state = MAIN_STATE_INIT;

    i2c_init();
    printf("i2c init.\n");

    pn532_init(PN532_ASYNC);
    printf("pn532 init.\n");

    if (pn532_getFirmwareVersion(cb_firmwareVersion))
    {
        printf("init, pn532_getFirmwareVersion failed.\n");
    }
    if (pn532_blockForCallback())
    {
        printf("init, pn532_blockForCallback failed.\n");
    }

    pn532_SAMConfiguration(1, 0, 1, cb_SAMConfiguration);
    pn532_blockForCallback();

    pn532_getGeneralStatus(cb_generalStatus);
    pn532_blockForCallback();
}

/* Loop
 * Does all of the main tasks.
 */
void loop()
{
    uint8_t success;
    uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
    uint8_t uidLength;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)

    wdt_reset();

    while (!usart_getchar(&in_char))
	{
        // Perfunctory stuff
        if (in_char == '.')
        {
//  			printf("\n\rUsage:");
//  			printf(":loud_cmd or [quiet_cmd]");
//  			printf("[door lock/unlock][dl][du] - [rm UID][-UID] - [add UID][+UID]");
            buffer_reset();
        }
        // Input state machine
        switch (input_state)
        {
        case 0: /* State 0: disregard input */
            if (in_char == ':')
            {
                buffer_reset();
                input_state = 1;
                printf("Command: ");
            }
            if (in_char == '[')
            {
                buffer_reset();
                input_state = 2;
            }
            break;
        case 1: /* State 1: load buffer, noisily echo input */
            if (in_char == '\n' || in_char == '\r' || buffer_index == BUF_LEN)
            {
                printf("%c", in_char);
                buffer_process(true);
                input_state = 0;
            }
            else if (in_char == ':')
            {
                buffer_reset();
                input_state = 1;
            }
            else if (in_char == '[')
            {
                buffer_reset();
                input_state = 2;
            }
            else if (in_char == 0)
            {
                buffer_reset();
                input_state = 0;
            }
            else
            {
                printf("%c", in_char);
                in_buffer[buffer_index++] = in_char;
            }
            break;
        case 2: /* State 2: load buffer silently */
            if (in_char == ']' || buffer_index == BUF_LEN)
            {
                buffer_process();
                input_state = 0;
            }
            else if (in_char == ':')
            {
                buffer_reset();
                input_state = 1;
            }
            else if (in_char == '[')
            {
                buffer_reset();
                input_state = 2;
            }
            else if (in_char == 0)
            {
                buffer_reset();
                input_state = 0;
            }
            else
            {
                in_buffer[buffer_index++] = in_char;
            }
            break;
        default:
            printf(" Error, switch: default case.");
            input_state=0;
            break;
        } // end switch
        if (buffer_index == 0 && input_state == 1)
        {
//  			printf("\n\r> ");
        }
        in_char = '\0';
    } // End if Serial.available()

    // Wait for an ISO14443A type cards (Mifare, etc.).  When one is found
    // 'uid' will be populated with the UID, and uidLength will indicate
    // if the uid is 4 bytes (Mifare Classic) or 7 bytes (Mifare Ultralight)
    if (use_nfc && !input_state)
    {
    	proc_pn532();
    }
}

/* Main
 * This function is used to override main() provided by the Arduino library.
 * It simply structures the other high-level functions.
 */
int main(void)
{
	init();

    setup();

    for (;;)
    {
        loop();
    }

    return 0;
}


