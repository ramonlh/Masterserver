#include <EEPROM.h>
#include <Wire.h>
#include <Arduino.h>  // for type definitions

//byte EEPROM1dir=0x50;  // EEPROM de la placa RTC NO USADO
byte EEPROM2dir=0x51;    // EEPROM 24LC512

void writeEEPROM(int deviceaddress, unsigned int eeaddress, byte data ) 
{    // escribe en EEPROM auxiliar 24LC512
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddress >> 8));   // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.write(data);
  Wire.endTransmission();
  delay(5);
}

byte readEEPROM(int deviceaddress, unsigned int eeaddress ) 
{  // lee de EEPROM auxiliar 24LC512
  byte rdata = 0xFF;
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddress >> 8));   // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.endTransmission();
  Wire.requestFrom(deviceaddress,1);
  if (Wire.available()) rdata = Wire.read();
  return rdata;
}

template <class T> int EEPROM_writeAnything(int ee, const T& value)
{
    const byte* p = (const byte*)(const void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++)
      writeEEPROM(EEPROM2dir, ee++, *p++);
    return i;
}

template <class T> int EEPROM_readAnything(int ee, T& value)
{
    byte* p = (byte*)(void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++)
      *p++ = readEEPROM(EEPROM2dir, ee++);
    return i;
}

//template <class T> int EEPROM_writeAnything(int ee, const T& value)
//{
//    const byte* p = (const byte*)(const void*)&value;
//    unsigned int i;
//    if (ee >=4096)
//      for (i = 0; i < sizeof(value); i++)
//        writeEEPROM(EEPROM2dir, ee++, *p++);
//    else
//      for (i = 0; i < sizeof(value); i++)
//        EEPROM.write(ee++, *p++);
//    return i;
//}
//
//template <class T> int EEPROM_readAnything(int ee, T& value)
//{
//    byte* p = (byte*)(void*)&value;
//    unsigned int i;
//    if (ee >=4096)
//      for (i = 0; i < sizeof(value); i++)
//        *p++ = readEEPROM(EEPROM2dir, ee++);
//    else
//      for (i = 0; i < sizeof(value); i++)
//        *p++ = EEPROM.read(ee++);
//    return i;
//}

