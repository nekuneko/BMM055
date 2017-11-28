#include "bmm055.h"


bmm055::bmm055 ()
{  
  controlRepZ          = read8(BMM055_ADDRESS, BMM055_CTRL_REP_Z);              
  controlRepXY         = read8(BMM055_ADDRESS, BMM055_CTRL_REP_XY);
  controlHighThreshold = read8(BMM055_ADDRESS, BMM055_CTRL_HIGH_THRESHOLD);    
  controlLowThreshold  = read8(BMM055_ADDRESS, BMM055_CTRL_LOW_THRESHOLD);   
  controlInt2          = read8(BMM055_ADDRESS, BMM055_CTRL_INT2);    
  controlInt1          = read8(BMM055_ADDRESS, BMM055_CTRL_INT1);    
  controlOp            = read8(BMM055_ADDRESS, BMM055_CTRL_OP);   
  controlPower         = read8(BMM055_ADDRESS, BMM055_CTRL_POWER);   
  interruptStatusReg   = read8(BMM055_ADDRESS, BMM055_INT_STATUS);     
  chipID               = read8(BMM055_ADDRESS, BMM055_CHIP_ID); 

 /*
  rawRHall = read16_LE(BMM055_ADDRESS, BMM055_RHALL_LSB);    
  rawDataZ = read16_LE(BMM055_ADDRESS, BMM055_DATAZ_LSB);    
  rawDataY = read16_LE(BMM055_ADDRESS, BMM055_DATAY_LSB);    
  rawDataX = read16_LE(BMM055_ADDRESS, BMM055_DATAX_LSB);  
  */
}


void bmm055::setRepZ (uint8_t value)
{
  controlRepZ = value;
  write8(BMM055_ADDRESS, BMM055_CTRL_REP_Z, controlRepZ);
  delay(2); // wait until changes take effect
}

void bmm055::setRepXY (uint8_t value)
{
  controlRepXY = value;
  write8(BMM055_ADDRESS, BMM055_CTRL_REP_XY, controlRepXY);
  delay(2); // wait until changes take effect
}

void bmm055::setHighThreshold (uint8_t value) 
{
  controlHighThreshold = value;
  write8(BMM055_ADDRESS, BMM055_CTRL_HIGH_THRESHOLD, controlHighThreshold);
  delay(2); // wait until changes take effect
}

void bmm055::setLowThreshold (uint8_t value) 
{
  controlLowThreshold = value;
  write8(BMM055_ADDRESS, BMM055_CTRL_LOW_THRESHOLD, controlLowThreshold);
  delay(2); // wait until changes take effect
}

void bmm055::setControlInt2 (uint8_t value)
{
  controlInt2 = value;
  write8(BMM055_ADDRESS, BMM055_CTRL_INT2, controlInt2);
  delay(2); // wait until changes take effect
}

void bmm055::setControlInt1 (uint8_t value)
{
  controlInt1 = value;
  write8(BMM055_ADDRESS, BMM055_CTRL_INT1, controlInt1);
  delay(2); // wait until changes take effect
}

void bmm055::setControlOp (uint8_t value)
{
  controlOp = value;
  write8(BMM055_ADDRESS, BMM055_CTRL_OP, controlOp);
  delay(2); // wait until changes take effect
}

void bmm055::doSuspendMode ()
{
  write8(BMM055_ADDRESS, BMM055_CTRL_POWER, 0x0);
  delay(2); // wait until changes take effect
}

void bmm055::doSleepMode ()
{
  write8(BMM055_ADDRESS, BMM055_CTRL_POWER, 0x1);
  delay(2); // wait until changes take effect
}

void bmm055::doSoftReset ()
{
  write8(BMM055_ADDRESS, BMM055_CTRL_POWER, 0x83);
  delay(2); // wait until changes take effect
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


void bmm055::printMagRepZ ()
{
  uint8_t repZ = read8(BMM055_ADDRESS, BMM055_CTRL_REP_Z);
  Serial.print("     repZ: 0x"); Serial.println(repZ, HEX);
  Serial.print("default is 0x"); Serial.println(0x00, HEX);
}

void bmm055::printMagRepXY ()
{
  uint8_t repXY = read8(BMM055_ADDRESS, BMM055_CTRL_REP_XY);
  Serial.print("    repXY: 0x"); Serial.println(repXY, HEX);
  Serial.print("default is 0x"); Serial.println(0x00, HEX);
}

void bmm055::printMagHighThreshold ()
{
  uint8_t highThreshold = read8(BMM055_ADDRESS, BMM055_CTRL_HIGH_THRESHOLD);
  Serial.print("highThreshold: 0x"); Serial.println(highThreshold, HEX);
  Serial.print("    default is 0x"); Serial.println(0x00, HEX);
}

void bmm055::printMagLowThreshold ()
{
  uint8_t lowThreshold = read8(BMM055_ADDRESS, BMM055_CTRL_LOW_THRESHOLD);
  Serial.print("lowThreshold: 0x"); Serial.println(lowThreshold, HEX);
  Serial.print("   default is 0x"); Serial.println(0x00, HEX);
}


// Control Interruption & Axis Register 2 Desglosado
void bmm055::printMagControlInt2 ()
{
  controlInt2 = read8(BMM055_ADDRESS, BMM055_CTRL_INT2);
  Serial.print("controlInt2 0x4E: 0x"); Serial.println(controlInt2, HEX);
  Serial.print("       default is 0x"); Serial.print(0x07, HEX);
  Serial.println(" - XYZ axis active, interrupts disabled");
  
  Serial.print("[7] En Data Ready Pin:  ");
  Serial.println((controlInt2 >> 7) & 1u, HEX);

  Serial.print("[6] En Interrupt Pin:   ");
  Serial.println((controlInt2 >> 6) & 1u, HEX);

  Serial.print("[5] En Channel Z:       ");
  Serial.println((controlInt2 >> 5) & 1u, HEX);

  Serial.print("[4] En Channel Y:       ");
  Serial.println((controlInt2 >> 4) & 1u, HEX);

  Serial.print("[3] En Channel X:       ");
  Serial.println((controlInt2 >> 3) & 1u, HEX);

  Serial.print("[2] DR Polarity:        ");
  Serial.println((controlInt2 >> 2) & 1u, HEX);

  Serial.print("[1] Interrupt Latch:    ");
  Serial.println((controlInt2 >> 1) & 1u, HEX);

  Serial.print("[0] Interrupt Polarity: ");
  Serial.println((controlInt2 >> 0) & 1u, HEX);
}


// Control Interruption Register 1 Desglosado
void bmm055::printMagControlInt1 () 
{
  controlInt1 = read8(BMM055_ADDRESS, BMM055_CTRL_INT1); 
  Serial.print("controlInt1 0x4D: 0x"); Serial.println(controlInt1, HEX);
  Serial.print("       default is 0x"); Serial.print(0x3f, HEX);
  Serial.println(" - all disabled");
  
  Serial.print("[7] En Data Overrun: ");
  Serial.println((controlInt1 >> 7) & 1u, HEX);

  Serial.print("[6] En Overflow Int: ");
  Serial.println((controlInt1 >> 6) & 1u, HEX);

  Serial.print("[5] En High Int Z:   ");
  Serial.println((controlInt1 >> 5) & 1u, HEX);

  Serial.print("[4] En High Int Y:   ");
  Serial.println((controlInt1 >> 4) & 1u, HEX);

  Serial.print("[3] En High Int X:   ");
  Serial.println((controlInt1 >> 3) & 1u, HEX);

  Serial.print("[2] En Low Int Z:    ");
  Serial.println((controlInt1 >> 2) & 1u, HEX);

  Serial.print("[1] En Low Int Y:    ");
  Serial.println((controlInt1 >> 1) & 1u, HEX);

  Serial.print("[0] En Low Int X:    ");
  Serial.println((controlInt1 >> 0) & 1u, HEX);
}

// Control Operation Register Desglosado
void bmm055::printMagControlOp ()
{
  controlOp = read8(BMM055_ADDRESS, BMM055_CTRL_OP); 
  Serial.print("controlOp: 0x"); Serial.println(controlOp, HEX);
  Serial.print("default is 0x"); Serial.print(0x06, HEX);
  Serial.println(" - advanced self test off, Output Data Rate 10Hz, Sleep OP Mode, self test off");
  
  Serial.print("[7] Adv. ST <1>:   ");
  Serial.println((controlOp >> 7) & 1u, HEX);

  Serial.print("[6] Adv. ST <0>:   ");
  Serial.println((controlOp >> 6) & 1u, HEX);

  Serial.print("[5] Data rate <2>: ");
  Serial.println((controlOp >> 5) & 1u, HEX);

  Serial.print("[4] Data rate <1>: ");
  Serial.println((controlOp >> 4) & 1u, HEX);

  Serial.print("[3] Data rate <0>: ");
  Serial.println((controlOp >> 3) & 1u, HEX);

  Serial.print("[2] Opmode <1>:    ");
  Serial.println((controlOp >> 2) & 1u, HEX);

  Serial.print("[1] Opmode <0>:    ");
  Serial.println((controlOp >> 1) & 1u, HEX);

  Serial.print("[0] Selft Test:    ");
  Serial.println((controlOp >> 0) & 1u, HEX);
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
  rawRHall = read16_LE(BMM055_ADDRESS, BMM055_RHALL_LSB) >> 2;
  rawDataZ = read16_LE(BMM055_ADDRESS, BMM055_DATAZ_LSB) >> 1;
  rawDataY = read16_LE(BMM055_ADDRESS, BMM055_DATAY_LSB) >> 3;
  rawDataX = read16_LE(BMM055_ADDRESS, BMM055_DATAX_LSB) >> 3;

  Serial.print("rawRHall: 0x");
  Serial.println(rawRHall, HEX);
  
  Serial.print("rawDataZ: 0x");
  Serial.println(rawDataZ, HEX);

  Serial.print("rawDataY: 0x");
  Serial.println(rawDataY, HEX);

  Serial.print("rawDataX: 0x");
  Serial.println(rawDataX, HEX);
}


void bmm055::printMagChipID ()
{
  chipID = read8(BMM055_ADDRESS, BMM055_CHIP_ID);
  Serial.print("     Chip ID: 0x"); Serial.println(chipID, HEX);
  Serial.println("default is: 0x32");
}




