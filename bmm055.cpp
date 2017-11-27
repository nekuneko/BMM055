#include <Arduino.h>
#include "bmm055.h"
#include "i2c_adafruit.h"

bmm055::bmm055 ()
{
  repZ  = 0;                // Reg Addr 0x52 def val 0x00 -> 
  repXY = 0;                // Reg Addr 0x51 def val 0x00 ->
  highThreshold = 0;        // Reg Addr 0x50 def val 0x00 ->
  lowThreshold  = 0;        // Reg Addr 0x4F def val 0x00 ->
  controlINT2 = 0;          // Reg Addr 0x4E def val 0x07 -> 
  controlINT1 = 0;          // Reg Addr 0x4D def val 0x3f ->
  controlOP = 0;            // Reg Addr 0x4C def val 0x06 -> Operation Mode, data rate 
  controlPower = 0;         // Reg Addr 0x4B def val 0x01 -> Power control, soft reset
  interruptStatusReg = 0;   // Reg Addr 0x4A def val 0x00 -> INTERRUPT_STATUS_REG
  rawRHall = 0;            // Reg Addr 0x49 + 0x48
  rawDataZ = 0;            // Reg Addr 0x47 + 0x46
  rawDataY = 0;            // Reg Addr 0x45 + 0x44
  rawDataX = 0;            // Reg Addr 0x43 + 0x42
  chipID = 0;              // Reg Addr 0x40 def val 0x32
}


// Imprime todos los registros del magnetómetro sin desglose de información
void bmm055::printMagDefaultValues ()
{ 
  for (byte reg=0x52; reg>=0x4A; --reg)
  {
    Serial.print("REG 0x");
    Serial.print(reg, HEX);
    Serial.print(": 0x");
    Serial.println(read8(BMM055_ADDRESS, reg), HEX);
  }
  
  // Chip ID
  Serial.print("REG 0x");
  Serial.print(BMM055_CHIP_ID, HEX);
  Serial.print(": 0x");
  Serial.println(read8(BMM055_ADDRESS, BMM055_CHIP_ID), HEX);
  
}



// ¡PROXIMO DÍA! IMPLEMENTAR LA FUNCION PARA VER EL ESTADO DEL CONTROL INT 2


// Control Interruption Register 1 Desglosado
void bmm055::printMagControlINT1 ()
{
  controlINT1 = read8(BMM055_ADDRESS, BMM055_CTRL_INT1); 
  Serial.print("controlINT1: 0x"); Serial.println(controlINT1, HEX);
  Serial.print("  default is 0x"); Serial.print(0x3f, HEX);
  Serial.println(" - all disabled");
  
  Serial.print("[7] En Data Overrun: ");
  Serial.println((controlINT1 >> 7) & 1u, HEX);

  Serial.print("[6] En Overflow Int: ");
  Serial.println((controlINT1 >> 6) & 1u, HEX);

  Serial.print("[5] En High Int Z:   ");
  Serial.println((controlINT1 >> 5) & 1u, HEX);

  Serial.print("[4] En High Int Y:   ");
  Serial.println((controlINT1 >> 4) & 1u, HEX);

  Serial.print("[3] En High Int X:   ");
  Serial.println((controlINT1 >> 3) & 1u, HEX);

  Serial.print("[2] En Low Int Z:    ");
  Serial.println((controlINT1 >> 2) & 1u, HEX);

  Serial.print("[1] En Low Int Y:    ");
  Serial.println((controlINT1 >> 1) & 1u, HEX);

  Serial.print("[0] En Low Int X:    ");
  Serial.println((controlINT1 >> 0) & 1u, HEX);
}

// Control Operation Register Desglosado
void bmm055::printMagControlOP ()
{
  controlOP = read8(BMM055_ADDRESS, BMM055_CTRL_OP); 
  Serial.print("controlOP: 0x"); Serial.println(controlOP, HEX);
  Serial.print("default is 0x"); Serial.print(0x06, HEX);
  Serial.println(" - advanced self test off, Output Data Rate 10Hz, Sleep OP Mode, self test off");
  
  Serial.print("[7] Adv. ST <1>:   ");
  Serial.println((controlOP >> 7) & 1u, HEX);

  Serial.print("[6] Adv. ST <0>:   ");
  Serial.println((controlOP >> 6) & 1u, HEX);

  Serial.print("[5] Data rate <2>: ");
  Serial.println((controlOP >> 5) & 1u, HEX);

  Serial.print("[4] Data rate <1>: ");
  Serial.println((controlOP >> 4) & 1u, HEX);

  Serial.print("[3] Data rate <0>: ");
  Serial.println((controlOP >> 3) & 1u, HEX);

  Serial.print("[2] Opmode <1>:    ");
  Serial.println((controlOP >> 2) & 1u, HEX);

  Serial.print("[1] Opmode <0>:    ");
  Serial.println((controlOP >> 1) & 1u, HEX);

  Serial.print("[0] Selft Test:    ");
  Serial.println((controlOP >> 0) & 1u, HEX);
}

// Control Power Register Desglosado
void bmm055::printMagPowerStatus ()
{
  controlPower = read8(BMM055_ADDRESS, BMM055_CTRL_POWER);
  Serial.print("controlPower: 0x"); Serial.println(controlPower, HEX);
  Serial.print("   default is 0x"); Serial.print(0x01, HEX);
  Serial.println(" - SPI4-wire mode, Sleep Mode");
  
  Serial.print("[7] Soft Reset '1':    ");
  Serial.println((controlPower >> 7) & 1u, HEX);

  Serial.print("[6] (fixed to 0)       ");
  Serial.println((controlPower >> 6) & 1u, HEX);

  Serial.print("[5] (fixed to 0)       ");
  Serial.println((controlPower >> 5) & 1u, HEX);

  Serial.print("[4] (fixed to 0)       ");
  Serial.println((controlPower >> 4) & 1u, HEX);

  Serial.print("[3] (fixed to 0)       ");
  Serial.println((controlPower >> 3) & 1u, HEX);

  Serial.print("[2] SPI3en:            ");
  Serial.println((controlPower >> 2) & 1u, HEX);

  Serial.print("[1] Soft Reset '1':    ");
  Serial.println((controlPower >> 1) & 1u, HEX);

  Serial.print("[0] Power Control Bit: ");
  Serial.println((controlPower >> 0) & 1u, HEX); // when 0, suspend mode is selected
}

// Interrupt Status Register Desglosado
void bmm055::printMagInterruptStatus ()
{
  interruptStatusReg = read8(BMM055_ADDRESS, BMM055_INT_STATUS);
  Serial.print("interruptStatusReg: 0x"); Serial.println(interruptStatusReg, HEX);
  Serial.print("         default is 0x"); Serial.println(0x00, HEX);
  
  Serial.print("[7] Data overrun: ");
  Serial.println((interruptStatusReg >> 7) & 1u, HEX);

  Serial.print("[6] Overflow:     ");
  Serial.println((interruptStatusReg >> 6) & 1u, HEX);

  Serial.print("[5] High Int Z:   ");
  Serial.println((interruptStatusReg >> 5) & 1u, HEX);

  Serial.print("[4] High Int Y:   ");
  Serial.println((interruptStatusReg >> 4) & 1u, HEX);

  Serial.print("[3] High Int X:   ");
  Serial.println((interruptStatusReg >> 3) & 1u, HEX);

  Serial.print("[2] Low Int Z:    ");
  Serial.println((interruptStatusReg >> 2) & 1u, HEX);

  Serial.print("[1] Low Int Y:    ");
  Serial.println((interruptStatusReg >> 1) & 1u, HEX);

  Serial.print("[0] Low Int X:    ");
  Serial.println((interruptStatusReg >> 0) & 1u, HEX);
  
}

// Imprime el valor raw de X, Y, Z y RHALL
void bmm055::printMagRawData ()
{
  rawRHall = read16_LE(BMM055_ADDRESS, 0x48) >> 2;
  rawDataZ = read16_LE(BMM055_ADDRESS, 0x46) >> 1;
  rawDataY = read16_LE(BMM055_ADDRESS, 0x44) >> 3;
  rawDataX = read16_LE(BMM055_ADDRESS, 0x42) >> 3;

  Serial.print("rawRHall: 0x");
  Serial.println(rawRHall, HEX);
  
  Serial.print("rawDataZ: 0x");
  Serial.println(rawDataZ, HEX);

  Serial.print("rawDataZ: 0x");
  Serial.println(rawDataZ, HEX);

  Serial.print("rawDataZ: 0x");
  Serial.println(rawDataZ, HEX);
}


void bmm055::printMagChipID ()
{
  chipID = read8(BMM055_ADDRESS, BMM055_CHIP_ID);
  Serial.print("Chip ID: 0x");
  Serial.println(chipID, HEX);
}


void bmm055::setSuspendMode ()
{
  write8(BMM055_ADDRESS, BMM055_CTRL_POWER, 0x0);
  delay(2); // asegurar escritura
}

void bmm055::setSleepMode ()
{
  write8(BMM055_ADDRESS, BMM055_CTRL_POWER, 0x1);
  delay(2); // asegurar escritura
}
