#include "i2c_adafruit.h"

/* i2c funcions by Adafruit */

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



uint8_t read8 (byte i2c_addr, byte reg)
{
  uint8_t value;

  Wire.beginTransmission((uint8_t)i2c_addr);
  Wire.write((uint8_t)reg);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)i2c_addr, (byte)1);
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
  value = (Wire.read() << 8) | Wire.read();


  return value;
}


uint16_t read16_LE(byte i2c_addr, byte reg) 
{
  uint16_t temp = read16(i2c_addr, reg);
  return (temp >> 8) | (temp << 8);

}


int16_t readS16(byte i2c_addr, byte reg)
{
  return (int16_t)read16(i2c_addr, reg);

}

int16_t readS16_LE(byte i2c_addr, byte reg)
{
  return (int16_t)read16_LE(i2c_addr, reg);

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
