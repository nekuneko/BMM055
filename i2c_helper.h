#ifndef _I2C_HELPER_H_
#define _I2C_HELPER_H_

#include <Arduino.h>
#include <Wire.h>

void i2c_init (); // Wire.begin();


void write8 (uint8_t i2c_addr, uint8_t reg, uint8_t value);

// Unsigned
uint8_t  read8     (uint8_t i2c_addr, uint8_t reg);
uint16_t read16    (uint8_t i2c_addr, uint8_t reg);
uint16_t read16_LE (uint8_t i2c_addr, uint8_t reg);
uint32_t read24    (uint8_t i2c_addr, uint8_t reg);
void     burstRead (uint8_t i2c_addr, uint8_t reg, uint8_t* values, uint8_t n_bytes);

// Signed
int8_t  readS8    (uint8_t i2c_addr, uint8_t reg); // New
int16_t readS16   (uint8_t i2c_addr, uint8_t reg);
int16_t readS16_LE(uint8_t i2c_addr, uint8_t reg);




#endif // _I2C_HELPER_H_