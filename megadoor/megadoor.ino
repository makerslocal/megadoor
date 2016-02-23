/**
 * MEGADOOR - It's a thing.
 *
 * This firmware is intended for an Atmega328P;
 * it will read NFC UIDs from an i2c connection
 * and unlock a door if the card is valid.
 */

/////////////
// HEADERS //
/////////////

// Included to keep compatibility between PIO and IDE
#include <Arduino.h>
// Included because something's broken in PIO
#include <Wire.h>
#include <SPI.h>
// Included for the firmware
#include <Adafruit_PN532.h>
#include <EEPROM.h>

//////////
// PINS //
//////////

// Required for I2C
#define PN532_IRQ   (6)
#define PN532_RESET (5)
// Trips the transistor switch
#define PIN_DOOR_UNLOCK (9) // Wired to NPN transistor
// Have Lassy fetch help!
#define PIN_TROUBLE (13) // Wired to LED
/* !I2C!
  There's an implied SDA&SCL pin here.
  The Adafruid_PN532 library uses the ATmega's
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

// Instantiate the NFC library
Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET);
// Set debugging
//#define DEBUG
// Set version MAJ.MIN-REV
#define VER_MAJ 0
#define VER_MIN 1
#define VER_REV 3

/**
 * setup() - a poem
 * We do what we must \
 * because they make us.
 */
void setup() {
  Serial.begin(9600);
  #ifdef DEBUG
  Serial.print("megadoor version "); Serial.println(VERSION);
  Serial.print("I have "); Serial.print(E2END); Serial.println(" bytes EEPROM.");
  #endif

  pinMode(PIN_DOOR_UNLOCK, OUTPUT);

  nfc.begin();
  uint32_t versiondata = nfc.getFirmwareVersion();
  if (! versiondata) {
    Serial.print("Didn't find PN53x board");
    while (1) // Sad blink
    {
      delay(1000);
      digitalWrite(PIN_TROUBLE, !digitalRead(PIN_TROUBLE));
    }
  }
  // Got ok data, print it out!
  #ifdef DEBUG
    Serial.print("Found chip PN5"); Serial.println((versiondata>>24) & 0xFF, HEX);
    Serial.print("Firmware ver. "); Serial.print((versiondata>>16) & 0xFF, DEC);
    Serial.print('.'); Serial.println((versiondata>>8) & 0xFF, DEC);
  #endif
  // configure board to read RFID tags
  nfc.SAMConfig();
  #ifdef DEBUG
    Serial.println("Waiting for an ISO14443A Card ...");
  #endif
}

void loop() {
  uint8_t success;
  uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
  uint8_t uidLength;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)

  // Wait for an ISO14443A type cards (Mifare, etc.).  When one is found
  // 'uid' will be populated with the UID, and uidLength will indicate
  // if the uid is 4 bytes (Mifare Classic) or 7 bytes (Mifare Ultralight)
  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);

  if (success)
  {
    // Display some basic information about the card
    #ifdef DEBUG
      Serial.println("Found an ISO14443A card");
      Serial.print("  UID Length: ");Serial.print(uidLength, DEC);Serial.println(" bytes");
      Serial.print("  UID Value: ");
      nfc.PrintHex(uid, uidLength);
      Serial.println("");
    #else
      nfc.PrintHex(uid, uidLength);
    #endif

    // Reference UID against known valid
    if ( checkCard(uid,uidLength) )
    {
      #ifdef DEBUG
        Serial.print("UID length "); Serial.print(uidLength); Serial.println(" accepted.");
      #endif
      digitalWrite(PIN_DOOR_UNLOCK, HIGH);
      delay(1000);
      digitalWrite(PIN_DOOR_UNLOCK, LOW);
    }
    else
    {
      #ifdef DEBUG
        Serial.print("UID length "); Serial.print(uidLength); Serial.println(" rejected.");
      #endif
      digitalWrite(PIN_TROUBLE, HIGH);
      delay(1000);
      digitalWrite(PIN_TROUBLE, LOW);
    }
  }
}


/*
  checkCard()

  Usage:
   byte testData[] = { 0x11,0x11,0x11,0x11,0x22,0x33,0x44 };
   if ( checkCard(testData,7) ) {
     Serial.println("ok");
   } else {
     Serial.println("no");
   }
 */
boolean checkCard(const byte* cardUid, uint8_t cardUidLength) {

  //Serial.print("Checking a uid of length "); Serial.println(cardUidLength);

  int e2idx = 0; //should be 0
  while ( e2idx < E2END ) {
    //Serial.print("Starting check at e2idx "); Serial.println(e2idx,DEC);
    int i = 0;
    while ( i < cardUidLength ) {
      if ( EEPROM.read(e2idx+i) != cardUid[i] ) {
        //Serial.print(EEPROM.read(e2idx+i),HEX); Serial.print(" Doesn't match "); Serial.println(cardUid[i],HEX);
        break;
      }
      //Serial.print(i,DEC); Serial.print(" "); Serial.println(cardUidLength,DEC);
      if ( ++i >= cardUidLength ) {
        //we got through the whole check without breaking - it must match
        //Serial.println("It matches!!!");
        Serial.print("Found at e2idx "); Serial.println(e2idx,DEC);
        return true;
      }
    }//end card while
    //Serial.print("No card found looking at "); Serial.println(e2idx,DEC);
    //delay(1000);

    //Serial.println("Finding next idx");
    while ( EEPROM.read(e2idx) != 0xff ) {
      //Serial.print(e2idx,DEC); Serial.print(" ");
      e2idx++;
    }
    e2idx++; //find the ff and then skip it
    //Serial.println("!");

  }//end e2 while

  return false;

}
