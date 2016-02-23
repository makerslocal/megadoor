#include <EEPROM.h>

void setup() {
  Serial.begin(9600);
  
  Serial.println("megadoor version 0.1");
  Serial.print("I have this much eeprom: ");
  Serial.println(E2END);
    
}
void loop() {
  
  byte testData[] = { 0x11,0x11,0x11,0x11,0x22,0x33,0x44 };
  if ( checkCard(testData,7) ) {
    Serial.println("ok");
  } else {
    Serial.println("no");
  }
  
  delay(10000);
  
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
