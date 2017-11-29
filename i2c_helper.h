#ifndef _I2C_HELPER_H_
#define _I2C_HELPER_H_

#include <Arduino.h>
#include <Wire.h>

void i2c_init (); // Wire.begin();


void write8 (byte i2c_addr, byte reg, byte value);

// Unsigned
uint8_t  read8     (byte i2c_addr, byte reg);
uint16_t read16    (byte i2c_addr, byte reg);
uint16_t read16_LE (byte i2c_addr, byte reg);
uint32_t read24    (byte i2c_addr, byte reg);

// Signed
int8_t  readS8    (byte i2c_addr, byte reg); // New
int16_t readS16   (byte i2c_addr, byte reg);
int16_t readS16_LE(byte i2c_addr, byte reg);



#endif // _I2C_HELPER_H_