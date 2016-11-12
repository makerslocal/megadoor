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
  - Combine serial input and state machine.
  - Implement external EEPROM
  - Organize EEPROM structure.
    I'm thinking a struct with
    configurable settings at the front,
    followed by a list or table of
    NFC and user data.
  - Implement LPM
  - Implement USB Host (to make USB keys work)
  - Implement BIGSIGN and/or RQ Welcome message
 */

/////////////
// HEADERS //
/////////////

// Included to keep compatibility between PIO and IDE
#include <Arduino.h>
// Included because something's broken in PIO
#include <Wire.h>
//#include <SPI.h>
// Included for the firmware
//#include <EEPROM.h>
// Include for Watchdog
#include <avr/wdt.h>
//#include <FastLED.h>
#include "sha1.h"
#include "keys.h"

// 4-Wire I2C
// Uncomment this and comment 2-Wire below to use Adafruit's latest libraries with IRQ.
//#include <Adafruit_PN532.h>

// 2-Wire I2C
// Uncomment this and comment 4-Wire above to use Adafruit's old libraries.
#include <PN532_I2C.h>
#include <PN532.h>
#include <NfcAdapter.h>
PN532_I2C pn532i2c(Wire);
PN532 nfc(pn532i2c);

//////////
// PINS //
//////////

// Required for I2C
#define PN532_IRQ   (6) // NOT CONNECTED
#define PN532_RESET (5) // NOT CONNECTED
// Trips the transistor switch
#define PIN_DOOR_UNLOCK (9) // Wired to NPN transistor
// Have Lassy fetch help!
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

// Instantiate the 4-Wire I2C NFC library
//Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET);

//bool debug = true; // Set debugging
//bool use_nfc = true; // Use NFC sensor

#define debug 1
#define use_nfc 1

// Set version MAJ.MIN-REV
#define VERSION "0.1.1-0"

#define BACKSPACE "\b \b"

// Serial buffer
const unsigned short int BUF_LEN = 16;
char in_char = '\0';
char in_buffer[BUF_LEN+1];
unsigned short buffer_index = 0;
short int input_state = 0; // Current state for reading serial input.
/* Serial State Machine
 * 0: Out of state, discard. ':' -> go to 1.
 * 1: Record command. '\n' -> go to 2.
 * 2: Process command and go to 0.
 */


////////////////
// PROTOTYPES //
////////////////

void watchdogSetup();
void watchdogTrip();
void door_unlock();
void door_lock();
//void uid_remove(uint8_t uid[], uint8_t uidLength, bool loud = false);
//void uid_add(uint8_t uid[], uint8_t uidLength, bool loud = false);
//int eeprom_byte_print(int addr, bool loud = false);
//void byte_set(int addr, uint8_t byte, bool loud = false);
//void eeprom_dump();
void buffer_reset();
void buffer_process(bool loud = false);

/**
 * setup() - a poem
 * We do what we must \
 * because they make us.
 */
void setup() {
  watchdogSetup();
  Serial.begin(115200);
  if (debug)
  {
    Serial.print("megadoor version "); Serial.println(VERSION);
    Serial.print("I have "); Serial.print(E2END); Serial.println(" bytes EEPROM.");
  }

  pinMode(PIN_DOOR_UNLOCK, OUTPUT);
  pinMode(PIN_TROUBLE, OUTPUT);
  digitalWrite(PIN_TROUBLE, LOW);
  digitalWrite(PIN_DOOR_UNLOCK, HIGH);

  nfc.begin();
  uint32_t versiondata = nfc.getFirmwareVersion();
  if (! versiondata) {
    Serial.print("Didn't find PN53x board");
    watchdogTrip(); // REMOVE FOR DEBUG WITHOUT PN532
    if (!debug) // Carry on if we're debugging.
    {
      watchdogTrip();
    }
//    } else {
//      use_nfc = false;
//    }
  }
  // Got ok data, print it out!
  if (debug&&use_nfc)
  {
    Serial.print("Found chip PN5"); Serial.println((versiondata>>24) & 0xFF, HEX);
    Serial.print("Firmware ver. "); Serial.print((versiondata>>16) & 0xFF, DEC);
    Serial.print('.'); Serial.println((versiondata>>8) & 0xFF, DEC);
  }

  // Set the max number of retry attempts to read from a card
  // This prevents us from waiting forever for a card, which is
  // the default behaviour of the PN532.
  if (use_nfc) nfc.setPassiveActivationRetries(0xFF); // For 2-Wire I2C only!

  // configure board to read RFID tags
  if (use_nfc) nfc.SAMConfig();

  if (debug&&use_nfc) Serial.println("Waiting for an ISO14443A Card ...");
}

void loop() {
  uint8_t success;
  uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
  uint8_t uidLength;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)

  wdt_reset();

  if (Serial.available())
  	{
  		in_char = Serial.read(); // Get the next character
      // Perfunctory stuff
  		if (in_char == '.') {
//  			Serial.println("\n\rUsage:");
//  			Serial.println(":loud_cmd or [quiet_cmd]");
//  			Serial.println("[door lock/unlock][dl][du] - [rm UID][-UID] - [add UID][+UID]");
        buffer_reset();
  		}
      // Input state machine
  		switch (input_state) {
  			case 0: /* State 0: disregard input */
  				if (in_char == ':') {
  					buffer_reset();
  					input_state = 1;
  				}
  				if (in_char == '[') {
  					buffer_reset();
  					input_state = 2;
  				}
  			break;
  			case 1: /* State 1: load buffer, noisily echo input */
  				if (in_char == '\n' || in_char == '\r' || buffer_index == BUF_LEN) {
            Serial.print(in_char);
  					buffer_process(true);
  					input_state = 0;
  				} else if (in_char == ':') {
  					buffer_reset();
  					input_state = 1;
  				} else if (in_char == '[') {
  					buffer_reset();
  					input_state = 2;
  				} else if (in_char == 0) {
  					Serial.println(". Input Cancelled.");
  					buffer_reset();
  					input_state = 0;
  				} else {
  					Serial.print(in_char);
  					in_buffer[buffer_index++] = in_char;
  				}
  			break;
  			case 2: /* State 2: load buffer silently */
  				if (in_char == ']' || buffer_index == BUF_LEN) {
  					buffer_process();
  					input_state = 0;
  				} else if (in_char == ':') {
  					buffer_reset();
  					input_state = 1;
  				} else if (in_char == '[') {
  					buffer_reset();
  					input_state = 2;
  				} else if (in_char == 0) {
  					buffer_reset();
  					input_state = 0;
  				} else {
  					in_buffer[buffer_index++] = in_char;
  				}
  			break;
  			default:
  				Serial.println(" Error, switch: default case.");
  				input_state=0;
  			break;
  		} // end switch
  		if (buffer_index == 0 && input_state == 1) {
  			Serial.print("\n\r> ");
  		}
  		in_char = '\0';
  	} // End if Serial.available()

  // Wait for an ISO14443A type cards (Mifare, etc.).  When one is found
  // 'uid' will be populated with the UID, and uidLength will indicate
  // if the uid is 4 bytes (Mifare Classic) or 7 bytes (Mifare Ultralight)
  if (use_nfc && !input_state)
  {
    if (debug) Serial.println(nfc.getFirmwareVersion(), HEX);
    uint32_t versiondata = nfc.getFirmwareVersion();
    if (versiondata == 0)
    {
      Serial.println("The NFC board has gone down!");
      watchdogTrip();
    }
    success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, 100);
    //if (debug) Serial.println("nfc read timeout/success.");

    if (success)
    {
      // Display some basic information about the card
      if (debug)
      {
//        Serial.println("Found an ISO14443A card");
        Serial.print("  UID Length: ");Serial.print(uidLength, DEC);Serial.println(" bytes");
        Serial.print("  UID Value: ");
        nfc.PrintHex(uid, uidLength);
        Serial.println("");
      }
      else
      {
        nfc.PrintHex(uid, uidLength);
      }

      // Reference UID against known valid
      if ( uid_check(uid,uidLength) >= 0 )
      {
        if (debug)
        {
          Serial.print("UID length "); Serial.print(uidLength); Serial.println(" accepted.");
        }
        door_unlock();
      }
      else
      {
        if (debug)
        {
          Serial.print("UID length "); Serial.print(uidLength); Serial.println(" rejected.");
        }
        digitalWrite(PIN_TROUBLE, HIGH);
        delay(50);
        digitalWrite(PIN_TROUBLE, LOW);
      }
    }
  }
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
  delay(10000);
}

void door_unlock ()
{
  wdt_reset();
  digitalWrite(PIN_DOOR_UNLOCK, LOW);
  delay(250);
  digitalWrite(PIN_DOOR_UNLOCK, HIGH);
  for (uint8_t i = 0; i < 6; i++)
  {
    wdt_reset();
    delay(500);
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
	if (loud) {
		Serial.print("\n\rProcessing: [");
		Serial.print(in_buffer);
    Serial.println("]");
	}
	if (in_buffer[0] == 'd') // Door
	{
    if (loud) Serial.print("Setting door: ");
		if (in_buffer[1] == 'u') // Door Unlock
		{
			if (loud) Serial.println("UNLOCKED.");
      //watchdogTrip();
      door_unlock();
		} // end Door Unlock
		else if (in_buffer[1] == 'l') // Door Lock
		{
			if (loud) Serial.println("LOCKED.");
      door_lock();
		} // end Door Lock
	} // end Door
//  else if (in_buffer[0] == 'u') // UID
//  {
//    if (loud) Serial.print("UID: ");
//    if (buffer_index != 17 && buffer_index != 11)
//    {
//      if (debug)
//      {
//        Serial.print(buffer_index);
//      }
//      Serial.println("Incorrect command/UID length.");
//      buffer_reset();
//      return;
//    }
//    if (in_buffer[1] == 'r') // UID Remove
//    {
//      if (loud) Serial.print(" REMOVE ");
//      uint8_t uid[7];
//      int addr;
//      char * endptr;
//      addr = strtol(&in_buffer[2], &endptr, 16);
//      uint8_t len = 0;
//      while (in_buffer[3+len] != '\0') // count uid length
//      {
//        len++;
//      }
//      len--;
//      Serial.println(len);
//      uint8_t idx;
//      for (idx = 0; idx < len; idx+=2)
//      {
//        char slice[3];
//        slice[0] = in_buffer[3+idx];
//        slice[1] = in_buffer[3+idx+1];
//        slice[2] = '\0';
//        uid[idx/2] = strtol(slice, nullptr, 16);
//      }
//      uid_remove(uid, len>10?7:4, loud);
//    } // end UID Remove
//    else if (in_buffer[1] == 'a') // UID Add
//    {
//      if (loud) Serial.print(" ADD ");
//      uint8_t uid[7];
//      int addr;
//      char * endptr;
//      addr = strtol(&in_buffer[2], &endptr, 16);
//      uint8_t len = 0;
//      while (in_buffer[3+len] != '\0') // count uid length
//      {
//        len++;
//      }
//      len--;
//      Serial.println(len);
//      uint8_t idx;
//      for (idx = 0; idx < len; idx+=2)
//      {
//        char slice[3];
//        slice[0] = in_buffer[3+idx];
//        slice[1] = in_buffer[3+idx+1];
//        slice[2] = '\0';
//        uid[idx/2] = strtol(slice, nullptr, 16);
//      }
//      uid_add(uid, len>10?7:4, loud);
//    } // end UID Add
//  }
//  else if (in_buffer[0] == 'b') // EEPROM BYTE
//  {
//    if (loud) Serial.print("BYTE: ");
//    if (in_buffer[1] == 'd') // EEPROM BYTE DUMP
//    {
//      if (loud) Serial.println("DUMP");
//      eeprom_dump();
//    } else if (in_buffer[1] == 'g') { // EEPROM BYTE GET
//      if (loud) Serial.println("GET");
//      int addr;
//      Serial.println(in_buffer);
//      addr = strtol(&in_buffer[2], nullptr, 0);
//      Serial.println(addr);
//      eeprom_byte_print(addr, loud);
//    } else if (in_buffer[1] == 's') { // EEPROM BYTE SET
//      if (loud) Serial.println("SET");
//      uint8_t byte;
//      int addr;
//      char * endptr;
//      addr = strtol(&in_buffer[2], &endptr, 0);
//      byte = strtol(endptr, nullptr, 0);
//      // hex_decode(&in_buffer[3], 3, &addr);
//      // hex_decode(&in_buffer[7], 2, (int*)&byte);
//      eeprom_byte_print(addr, loud);
//      Serial.println("-->");
//      EEPROM.update(addr, byte);
//      eeprom_byte_print(addr, loud);
//    }
//  }
	buffer_reset();
} // end buffer_process()

//int eeprom_byte_print(int addr, bool loud /*= false*/)
//{
//  uint8_t byte = EEPROM.read(addr);
//  if (loud||debug) {
//    Serial.print("Address [0x"); Serial.print(addr, HEX);
//    Serial.print("] Data: [0x"); Serial.print(byte, HEX); Serial.println("]");
//  }
//  return byte;
//}

//void byte_set(int addr, uint8_t byte, bool loud /*= false*/)
//{
//  if (loud||debug) {
//    Serial.print("Address [0x"); Serial.print(addr, HEX);
//    Serial.print("] Old data: [0x"); Serial.print(byte, HEX); Serial.println("]");
//  }
//  EEPROM.update(addr, byte);
//  if (loud||debug) {
//    Serial.print("New data: [0x"); Serial.print(EEPROM.read(addr), HEX); Serial.println("]");
//  }
//}

//void eeprom_dump()
//{
//  int addr;
//  for (addr = 0; addr < E2END; addr+=8)
//  {
//    if (addr%100 == 0)
//      wdt_reset();
//    uint8_t subaddr;
//    Serial.print("Addr [0x"); Serial.print(addr, HEX);
//    Serial.print("-0x"); Serial.print(addr+7, HEX);
//    Serial.print("]\t");
//    for (subaddr = 0; subaddr < 8; subaddr++)
//    {
//      Serial.print("[0x"); Serial.print(EEPROM.read(addr+subaddr), HEX); Serial.print("] ");
//    }
//    Serial.print("\n\r");
//  }
//}

//void uid_remove(uint8_t uid[], uint8_t uidLength, bool loud /*= false*/)
//{
//  if (debug) {
//    Serial.print("UID length: ");
//    Serial.println(uidLength);
//    Serial.print("UID to remove: ");
//    nfc.PrintHex(uid, uidLength);
//  }
//  // Get the EEPROM address of the UID
//  int addr = uid_check(uid, uidLength);
//  if (addr < 0) // UID doesn't exist in EEPROM
//  {
//    if (loud||debug) Serial.println("UID already vacant.");
//    return;
//  } else {
//    if (debug) {
//      Serial.print("Found UID in eeprom. Address: ");
//      Serial.println(addr);
//    }
//  }
//  // Purge the UID
//  uint8_t index;
//  // Check that a trailing 0xff exists.
//  // Otherwise we may delete the wrong size UID.
//  if (EEPROM.read(addr+uidLength) != 0xff)
//  {
//    Serial.print("There is a problem. The terminating 0xff is absent. Aborting! EEPROM decimal address: ");
//    Serial.println(addr+uidLength, DEC);
//    return;
//  }
//  for (index = 0; index < uidLength; ++index)
//  {
//    // Check that there isn't a terminating 0xff in the middle of our UID.
//    if (EEPROM.read(addr+index) == 0xff)
//    {
//      Serial.print("There is a problem. Part of the UID was 0xff. Aborting! EEPROM decimal address: ");
//      Serial.println(addr+index, DEC);
//      return;
//    }
//    // Check again that the byte we're erasing matches the UID.
//    if (EEPROM.read(addr+index) != uid[index])
//    {
//      Serial.print("There is a problem. The UID doesn't match what we're erasing. Aborting! EEPROM decimal address: ");
//      Serial.println(addr+index, DEC);
//      return;
//    }
//    if (loud||debug)
//    {
//      Serial.print("Updating ");
//      Serial.print((addr+index), DEC);
//      Serial.println(" to 0x00");
//    }
//    EEPROM.update(addr+index, 0x00);
//  }
//}

//void uid_add(uint8_t uid[], uint8_t uidLength, bool loud /*= false*/)
//{
//  // Get the EEPROM address of the UID
//  int addr = uid_check(uid, uidLength);
//  if (debug) {
//    Serial.print("check address: ");
//    Serial.println(addr, HEX);
//  }
//  if (addr >= 0) // UID is already in EEPROM
//  {
//    if (loud||debug) Serial.println("UID already exists.");
//    return;
//  }
//  // Create UID in EEPROM
//  uint8_t blank[] = { 0, 0, 0, 0, 0, 0, 0 }; // Oh. Oh man. This is sneaky.
//  // Get the EEPROM address of an empty spot
//  addr = uid_check(blank, uidLength);
//  if (addr >= 0) // Found
//  {
//    int index;
//    for (index = 0; index < uidLength; ++index)
//    {
//      // Check that there isn't a terminating 0xff in the middle of our UID.
//      if (EEPROM.read(addr+index) == 0xff)
//      {
//        Serial.print("There is a problem. Part of the UID was 0xff. Aborting! EEPROM decimal address: ");
//        Serial.println(addr+index, HEX);
//        return;
//      }
//      if (loud||debug)
//      {
//        Serial.print("Updating ");
//        Serial.print((addr+index), HEX);
//        Serial.print(" to ");
//        Serial.println(uid[index], HEX);
//      }
//      EEPROM.update(addr+index, uid[index]);
//    }
//    EEPROM.update(addr+index, 0xff);
//  }
//  else // No empty spots
//  {
//    // Iterate until two adjacent bytes are 0xff
//    for (addr = 1; addr <= E2END; ++addr)
//    {
//      if (EEPROM.read(addr) == 0xff && EEPROM.read(addr-1) == 0xff) // Match
//      {
//        int index;
//        for (index = 0; index < uidLength; ++index)
//        {
//          if (loud||debug)
//          {
//            Serial.print("Updating ");
//            Serial.print((addr+index), HEX);
//            Serial.print(" to ");
//            Serial.println(uid[index], HEX);
//          }
//          EEPROM.update(addr+index, uid[index]);
//        }
//        EEPROM.update(addr+index, 0xff);
//        return;
//      }
//    }
//    Serial.println(":c no dice.");
//  }
//}

int uid_check(uint8_t* uid, uint8_t uidLength) {
  /* Usage:
     byte testData[] = { 0x11,0x11,0x11,0x11,0x22,0x33,0x44 };
     if ( uid_check(testData,7) >= 0 ) {
       Serial.println("ok");
     } else {
       Serial.println("no");
     }
  */

  /*if (debug) {
    Serial.print("Checking a uid of length "); Serial.println(uidLength);
  }*/
  
  uint8_t * sha_key;
  Sha1.init();
//  Sha1.print(text);

  for (uint8_t ind = 0; ind < uidLength; ind++)
  {
    char c[4];
    sprintf(c, "%02x", uid[ind]);
//    text[ind] = uid[ind];
    Sha1.write(c);
  }

  sha_key = Sha1.result();
  uint8_t match_flag = 1;

  Serial.print("Trying to match sha1: ");
  for (uint8_t ind = 0; ind < 20; ind++)
  {
    Serial.print((char)sha_key[ind]);
    Serial.print(",");
  }

  for (uint16_t index = 0; index < NUM_KEYS; index++)
  {
    match_flag = 1;
    Serial.println("=======");
    Serial.println(index);
    for (uint8_t key_index = 0; key_index < KEY_LEN; key_index++)
    {
      Serial.print("[");
      Serial.print(flash_storage[index][key_index]);
      Serial.print(", ");
      Serial.print(sha_key[key_index]);
      Serial.println("]");
      if (flash_storage[index][key_index] != sha_key[key_index])
      {
        match_flag = 0;
        break;
      }
    }
    if (match_flag)
    {
      return(0);
    }
  }



//  int e2idx = 0; //should be 0
//  while ( e2idx <= E2END ) {
//    /*if (debug) {
//      Serial.print("Starting check at e2idx "); Serial.println(e2idx,DEC);
//    }*/
//
//    uint8_t i = 0;
//    while ( i < uidLength ) {
//      if ( EEPROM.read(e2idx+i) != uid[i] ) {
//        /*if (debug) {
//          Serial.print(EEPROM.read(e2idx+i),HEX); Serial.print(" Doesn't match "); Serial.println(uid[i],HEX);
//        }*/
//        break;
//      }
//      /*if (debug) {
//        Serial.print(i,DEC); Serial.print(" "); Serial.println(uidLength,DEC);
//      }*/
//      i++;
//      if ( i >= uidLength && EEPROM.read(e2idx+i) == 0xff) {
//        //we got through the whole check without breaking - it must match
//        //Serial.println("It matches!!!");
//        // if (EEPROM.read(e2idx+i) != 0xff) // The terminator is missing
//        // {
//        //
//        // }
//        Serial.print("Found at e2idx "); Serial.println(e2idx,DEC);
//        return e2idx;
//      }
//    }//end card while
//
//    /*if (debug) {
//      Serial.print("No card found looking at "); Serial.println(e2idx,DEC);
//      //delay(1000);
//    }*/
//
//    //if (debug) Serial.println("Finding next idx");
//    while ( EEPROM.read(e2idx) != 0xff ) {
//      /*if (debug) {
//        Serial.print(e2idx,DEC); Serial.print(" ");
//      }*/
//      e2idx++;
//    }
//    e2idx++; //find the ff and then skip it
//    //if (debug) Serial.println("!");
//
//  }//end e2 while
//
//  return(-1);
return(-1);
} // end uid_check()
