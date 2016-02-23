#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_PN532.h>
#include <EEPROM.h>

// If using the breakout with SPI, define the pins for SPI communication.
//#define PN532_SCK  (2)
//#define PN532_MOSI (3)
//#define PN532_SS   (4)
//#define PN532_MISO (5)

// If using the breakout or shield with I2C, define just the pins connected
// to the IRQ and reset lines.  Use the values below (2, 3) for the shield!
#define PN532_IRQ   (6)
#define PN532_RESET (5)  // Not connected by default on the NFC Shield

// Uncomment just _one_ line below depending on how your breakout or shield
// is connected to the Arduino:

// Use this line for a breakout with a software SPI connection (recommended):
//Adafruit_PN532 nfc(PN532_SCK, PN532_MISO, PN532_MOSI, PN532_SS);

// Use this line for a breakout with a hardware SPI connection.  Note that
// the PN532 SCK, MOSI, and MISO pins need to be connected to the Arduino's
// hardware SPI SCK, MOSI, and MISO pins.  On an Arduino Uno these are
// SCK = 13, MOSI = 11, MISO = 12.  The SS line can be any digital IO pin.
//Adafruit_PN532 nfc(PN532_SS);

// Or use this line for a breakout or shield with an I2C connection:
Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET);


void setup() {
  Serial.begin(9600);

  Serial.println("megadoor version 0.1-2");
  Serial.print("I have this much eeprom: ");
  Serial.println(E2END);

  pinMode(9, OUTPUT);

  Serial.println("Hello!");

  nfc.begin();

  uint32_t versiondata = nfc.getFirmwareVersion();
  if (! versiondata) {
    Serial.print("Didn't find PN53x board");
    while (1); // halt
  }
  // Got ok data, print it out!
  Serial.print("Found chip PN5"); Serial.println((versiondata>>24) & 0xFF, HEX);
  Serial.print("Firmware ver. "); Serial.print((versiondata>>16) & 0xFF, DEC);
  Serial.print('.'); Serial.println((versiondata>>8) & 0xFF, DEC);

  // configure board to read RFID tags
  nfc.SAMConfig();

  Serial.println("Waiting for an ISO14443A Card ...");
}

void loop() {

//  byte testData[] = { 0x11,0x11,0x11,0x11,0x22,0x33,0x44 };
//  if ( checkCard(testData,7) ) {
//    Serial.println("ok");
//  } else {
//    Serial.println("no");
//  }
//
//  delay(10000);


  uint8_t success;
  uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
  uint8_t uidLength;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)

  // Wait for an ISO14443A type cards (Mifare, etc.).  When one is found
  // 'uid' will be populated with the UID, and uidLength will indicate
  // if the uid is 4 bytes (Mifare Classic) or 7 bytes (Mifare Ultralight)
  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);

  if (success) {
    // Display some basic information about the card
    Serial.println("Found an ISO14443A card");
    Serial.print("  UID Length: ");Serial.print(uidLength, DEC);Serial.println(" bytes");
    Serial.print("  UID Value: ");
    nfc.PrintHex(uid, uidLength);
    Serial.println("");

    if (uidLength == 4)
    {
      if ( checkCard(uid,4) ) {
        Serial.println("ok, 4 byte uid");
        digitalWrite(9, HIGH);
        delay(1000);
        digitalWrite(9, LOW);
      } else {
        Serial.println("nope 4.");
      }

//      // We probably have a Mifare Classic card ...
//      Serial.println("Seems to be a Mifare Classic card (4 byte UID)");
//
//      // Now we need to try to authenticate it for read/write access
//      // Try with the factory default KeyA: 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF
//      Serial.println("Trying to authenticate block 4 with default KEYA value");
//      uint8_t keya[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
//
//    // Start with block 4 (the first block of sector 1) since sector 0
//    // contains the manufacturer data and it's probably better just
//    // to leave it alone unless you know what you're doing
//      success = nfc.mifareclassic_AuthenticateBlock(uid, uidLength, 4, 0, keya);
//
//      if (success)
//      {
//        Serial.println("Sector 1 (Blocks 4..7) has been authenticated");
//        uint8_t data[16];
//
//        // If you want to write something to block 4 to test with, uncomment
//    // the following line and this text should be read back in a minute
//        //memcpy(data, (const uint8_t[]){ 'a', 'd', 'a', 'f', 'r', 'u', 'i', 't', '.', 'c', 'o', 'm', 0, 0, 0, 0 }, sizeof data);
//        // success = nfc.mifareclassic_WriteDataBlock (4, data);
//
//        // Try to read the contents of block 4
//        success = nfc.mifareclassic_ReadDataBlock(4, data);
//
//        if (success)
//        {
//          // Data seems to have been read ... spit it out
//          Serial.println("Reading Block 4:");
//          nfc.PrintHexChar(data, 16);
//          Serial.println("");
//
//          // Wait a bit before reading the card again
//          delay(1000);
//        }
//        else
//        {
//          Serial.println("Ooops ... unable to read the requested block.  Try another key?");
//        }
//      }
//      else
//      {
//        Serial.println("Ooops ... authentication failed: Try another key?");
//      }
    }

    if (uidLength == 7)
    {
      if ( checkCard(uid,7) ) {
        Serial.println("ok, 7 byte uid");
        digitalWrite(9, HIGH);
        delay(1000);
        digitalWrite(9, LOW);
      } else {
        Serial.println("nope 7.");
      }
//      // We probably have a Mifare Ultralight card ...
//      Serial.println("Seems to be a Mifare Ultralight tag (7 byte UID)");
//
//      for (int xi = 0; xi < 45; xi++)
//      {
//        // Try to read the first general-purpose user page (#4)
//        Serial.print("Reading page ");
//        Serial.println(xi);
//        uint8_t data[32];
//        success = nfc.mifareultralight_ReadPage (xi, data);
//        if (success)
//        {
//          // Data seems to have been read ... spit it out
//          nfc.PrintHexChar(data, 4);
//          //Serial.println("");
//
//          // Wait a bit before reading the card again
//          delay(10);
//        }
//        else
//        {
//          Serial.println("Ooops ... unable to read the requested page!?");
//        }
//      }
      delay(1000);
    }
  }

}

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
