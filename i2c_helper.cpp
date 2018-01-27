#include "i2c_helper.h"


void i2c_init ()
{
  Wire.begin();
}



void write8 (uint8_t i2c_addr, uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(i2c_addr);   // start transmission to device 
  Wire.write(reg);                    // sends register address to read from
  Wire.write(value);                  // write data
  Wire.endTransmission();             // end transmission
}


// Unsigned
uint8_t read8 (uint8_t i2c_addr, uint8_t reg)
{
  uint8_t value;

  Wire.beginTransmission(i2c_addr);   // start transmission to device 
  Wire.write(reg);                    // sends register address to read from
  Wire.endTransmission();             // end transmission

  Wire.beginTransmission(i2c_addr);   // start transmission to device 
  Wire.requestFrom(i2c_addr, 1u);     // send data n-bytes read
  //while(Wire.available()!=1u);        // wait until data is ready
  value = Wire.read();                  // receive DATA
  Wire.endTransmission();             // end transmission

  return value;
}



uint16_t read16(uint8_t i2c_addr, uint8_t reg)
{
  uint16_t value;

  Wire.beginTransmission(i2c_addr);   // start transmission to device 
  Wire.write(reg);                    // sends register address to read from
  Wire.endTransmission();             // end transmission

  Wire.beginTransmission(i2c_addr);   // start transmission to device 
  Wire.requestFrom(i2c_addr, 2u);     // send data n-bytes read
  //while(Wire.available()!=2u);      // wait until data is ready
  value = (Wire.read() << 8) | Wire.read();  // first receive MSB, secondly receive LSB
  Wire.endTransmission();             // end transmission

  return value;                       // return MSB + LSB
}


uint16_t read16_LE(uint8_t i2c_addr, uint8_t reg) 
{
  uint16_t temp = read16(i2c_addr, reg);  // temp = LSB + MSB
  return (temp >> 8) | (temp << 8);       // MSB + LSB
}


// REVISAR
uint32_t read24(uint8_t i2c_addr, uint8_t reg)
{
  uint32_t value;

  Wire.beginTransmission(i2c_addr);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.beginTransmission(i2c_addr);
  Wire.requestFrom(i2c_addr, 3u);
  //while(!Wire.available());
  value = Wire.read();
  value <<= 8;
  value |= Wire.read();
  value <<= 8;
  value |= Wire.read();
  Wire.endTransmission();             // end transmission

  return value;
}

void burstRead(uint8_t i2c_addr, uint8_t reg, uint8_t* values, uint8_t n_bytes)
{
  uint8_t value;
  
  Wire.beginTransmission(i2c_addr);     // start transmission to device 
  Wire.write(reg);                      // sends register address to read from
  Wire.endTransmission();               // end transmission

  Wire.beginTransmission(i2c_addr);     // start transmission to device 
  Wire.requestFrom(i2c_addr, n_bytes);  // send data n-bytes read
  //while(Wire.available()!=n_bytes);     // wait until data is ready
  for (int i=0; i<n_bytes; ++i)         // read data
    values[i] = Wire.read();
  Wire.endTransmission();
}

// Signed

int8_t readS8 (uint8_t i2c_addr, uint8_t reg)
{
  return (int8_t) read8(i2c_addr, reg);
}


int16_t readS16(uint8_t i2c_addr, uint8_t reg)
{
  return (int16_t) read16(i2c_addr, reg);
}

int16_t readS16_LE(uint8_t i2c_addr, uint8_t reg)
{
  return (int16_t) read16_LE(i2c_addr, reg);
}
