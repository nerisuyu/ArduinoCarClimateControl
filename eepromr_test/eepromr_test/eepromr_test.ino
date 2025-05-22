/*
 *  This sketch shows sector hoping across reboots
 */

#include <EEPROM_Rotate.h>

EEPROM_Rotate EEPROMr;

#define DATA_OFFSET     10
#define DATA2_OFFSET     11
#define INTDATA_OFFSET     4


int EepromReadInt(int ADDR){ //writes an integer to EEPROM Rotate
  int data=0;
  for(int i=0;i<4;++i){
    uint8_t byte = EEPROMr.read(ADDR+i);
    data=data+(byte<<(8*i));
    Serial.printf(" eepromReadInt: data is: %d\n",data);
  }   
  return data;
}

void EepromWriteInt(int ADDR,int data){ //reads an integer from EEPROM Rotate
  for(int i=0;i<4;++i){
    uint8_t byte = data&0xFF;
    data=data>>8;
    EEPROMr.write(ADDR+i,byte);
  }
}


void setup() {

    // DEBUG -------------------------------------------------------------------

    Serial.begin(9600);
    
    Serial.println();
    Serial.println();

    // EEPROM Initialization ---------------------------------------------------

    EEPROMr.size(4);
    EEPROMr.begin(4096);
    delay(2000);

    // Example -----------------------------------------------------------------

    // uint8_t data;

    // Serial.println();
    // Serial.printf("Position 0: 0x%02X\n", EEPROMr.read(0));
    // Serial.printf("Position 1: 0x%02X\n", EEPROMr.read(1));
    // Serial.printf("Position 2: 0x%02X\n", EEPROMr.read(2));
    // Serial.printf("Data      : 0x%02X\n", data = EEPROMr.read(DATA_OFFSET));

    // Serial.println();
    // Serial.printf("Writing 0x%02X to data\n", data + 1);
    // EEPROMr.write(DATA_OFFSET, data + 1);

    // Serial.println();
    // Serial.printf("Commit %s\n", EEPROMr.commit() ? "OK" : "KO");
    // Serial.printf("Position 0: 0x%02X\n", EEPROMr.read(0));
    // Serial.printf("Position 1: 0x%02X\n", EEPROMr.read(1));
    // Serial.printf("Position 2: 0x%02X\n", EEPROMr.read(2));
    // Serial.printf("Data      : 0x%02X\n", data = EEPROMr.read(DATA_OFFSET));

}

int intdata=1000;

void loop() {
  delay(2000);

  Serial.printf("Writing %d to intdata\n", intdata-160 );
  EepromWriteInt(INTDATA_OFFSET,intdata-160);
  Serial.printf("Commit %s\n", EEPROMr.commit() ? "OK" : "KO");
  delay(2000);

  Serial.printf("intdata is %d \n",intdata=EepromReadInt(INTDATA_OFFSET));
  Serial.println();
  // uint8_t data;
  // uint8_t data2;

  // Serial.println();
  // Serial.printf("Data    %d  \n", data = EEPROMr.read(DATA_OFFSET));
  // Serial.printf("Data2   %d   \n", data2 = EEPROMr.read(DATA2_OFFSET));

  // Serial.println();
    
  // Serial.printf("Writing %d to data\n", data );
  // Serial.printf("Writing %d to data2\n", data2 );
  // EEPROMr.write(DATA_OFFSET, data );
  // EEPROMr.write(DATA2_OFFSET, data2 );

  //Serial.printf("Data      : 0x%02X\n", data = EEPROMr.read(DATA_OFFSET));
}