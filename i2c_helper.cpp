#include "i2c_helper.h"


void i2c_init ()
{
  Wire.begin();
}



void write8 (byte i2c_addr, byte reg, byte value)
{
  Wire.beginTransmission((uint8_t)i2c_addr);
  Wire.write((uint8_t)reg);
  Wire.write((uint8_t)value);
  Wire.endTransmission();
}


// Unsigned
void burstRead(byte i2c_addr, byte reg, uint8_t* values, int n_bytes)
{
  uint8_t value;
  
  Wire.beginTransmission((uint8_t)i2c_addr);
  Wire.write((uint8_t)reg);
  Wire.endTransmission();

  Wire.requestFrom((uint8_t)i2c_addr, (byte)n_bytes);
  int i = 0;
  while(!Wire.available());
  while(Wire.available())
  {
    values[i] = Wire.read();
    ++i;
  }
    
}


uint8_t read8 (byte i2c_addr, byte reg)
{
  uint8_t value;

  Wire.beginTransmission((uint8_t)i2c_addr);
  Wire.write((uint8_t)reg);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)i2c_addr, (byte)1);

  //while(!Wire.available());
  value = Wire.read();

  return value;
}



uint16_t read16(byte i2c_addr, byte reg)
{
  uint16_t value;

  Wire.beginTransmission((uint8_t)i2c_addr);
  Wire.write((uint8_t)reg);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)i2c_addr, (byte)2);
  
  while(!Wire.available());
  value = (Wire.read() << 8) | Wire.read();


  return value;
}


uint16_t read16_LE(byte i2c_addr, byte reg) 
{
  uint16_t temp = read16(i2c_addr, reg);
  return (temp >> 8) | (temp << 8);
}

uint32_t read24(byte i2c_addr, byte reg)
{
  uint32_t value;

  Wire.beginTransmission((uint8_t)i2c_addr);
  Wire.write((uint8_t)reg);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)i2c_addr, (byte)3);
  
  value = Wire.read();
  value <<= 8;
  value |= Wire.read();
  value <<= 8;
  value |= Wire.read();

  return value;
}


// Signed
/*
void burstRead(byte i2c_addr, byte reg, int8_t* values, int n_bytes)
{
  uint8_t value;
  
  Wire.beginTransmission((uint8_t)i2c_addr);
  Wire.write((uint8_t)reg);
  Wire.endTransmission();

  Wire.requestFrom((uint8_t)i2c_addr, (byte)n_bytes);
  int i = 0;
  while(Wire.available())
  {
    values[i] = Wire.read();
    ++i;
  }
}
*/

int8_t readS8 (byte i2c_addr, byte reg)
{
  return (int8_t)read8(i2c_addr, reg);
}


int16_t readS16(byte i2c_addr, byte reg)
{
  return (int16_t)read16(i2c_addr, reg);
}

int16_t readS16_LE(byte i2c_addr, byte reg)
{
  return (int16_t)read16_LE(i2c_addr, reg);
}



