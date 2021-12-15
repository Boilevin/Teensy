#include "flashmem.h"
#include "drivers.h"
#include <EEPROM.h>
#include "mower.h"


FlashClass Flash;


int eereadwriteString(boolean readflag, int &ee, String& value)
{
  unsigned int i;
  if (readflag) {
    value = "";
    char ch = Flash.read(ee++);
    while (ch) {
      value += ch;
      ch = Flash.read(ee++);
    }
  } else {
    for(i=0; i<value.length(); i++) {
      Flash.write(ee++, value.charAt(i));
    }
    Flash.write(ee++, 0);
  }
}


FlashClass::FlashClass() {
  verboseOutput = false;
#ifdef __AVR__  
#else
#endif
}


void FlashClass::test(){    
  Serial.println(F("EEPROM test - Please wait..."));
  boolean success = true;
  for (int i=0; i < 1024; i++){ // test 1024 addresses
    byte temp = read(i);  // read original value
    write(i, ((byte)i));  // write test value
    byte v = read(i); // get test value
    write(i, temp); // write back original value
    if (v != ((byte)i)){ // incorrect read or write or both
      Serial.println(F("EEPROM error - RTC module missing?"));
      success = false;
      break;
    }   
  }
  if (success) Serial.println(F("success!"));  
}

byte FlashClass::read(uint32_t address) {
  
  return EEPROM.read(address);

}

byte* FlashClass::readAddress(uint32_t address) {
 
  byte d = EEPROM.read(address);
  return &d;

}

void FlashClass::dump(){
  Serial.println(F("EEPROM dump"));
  for (int i=0; i < 1024; i++){
    byte v = read(i);
    if (v != 0){
      Serial.print(i);
      Serial.print(F("="));
      Serial.print(v);
      Serial.print(F(", "));
    }
  }
  Serial.println();
}

boolean FlashClass::write(uint32_t address, byte value) {
  if (verboseOutput){
    Serial.print(F("!76,"));
    Serial.print(address);
    Serial.print(F(","));
    Serial.print(value);  
    Serial.println();
  }
 
  EEPROM.write(address, value);
  return true;

}


boolean FlashClass::write(uint32_t address, byte *data, uint32_t dataLength) {
  for (int i=0; i < dataLength; i++){
    write(address + i, data[i]);
  }
  return true;
}
