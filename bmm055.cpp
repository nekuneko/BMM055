#include "bmm055.h"

// REVISAR! si algo está incompleto

bmm055::bmm055 ()
{}

bmm055::init ()
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

  init_trim_registers();
}

void bmm055::init_trim_registers ()
{
  dig_x1 = readS8(BMM055_ADDRESS, BMM050_DIG_X1);              // < trim x1 data
  dig_y1 = readS8(BMM055_ADDRESS, BMM050_DIG_Y1);              // < trim y1 data 

  dig_x2 = readS8(BMM055_ADDRESS, BMM050_DIG_X2);              // < trim x2 data 
  dig_y2 = readS8(BMM055_ADDRESS, BMM050_DIG_Y2);              // < trim y2 data

  dig_xy1 = read8 (BMM055_ADDRESS, BMM050_DIG_XY1);           // < trim xy1 data 
  dig_xy2 = readS8(BMM055_ADDRESS, BMM050_DIG_XY2);           // < trim xy2 data

  dig_z1 = read16_LE (BMM055_ADDRESS, BMM050_DIG_Z1_LSB);    // < trim z1 data 
  dig_z2 = readS16_LE(BMM055_ADDRESS, BMM050_DIG_Z2_LSB);    // < trim z2 data 
  dig_z3 = readS16_LE(BMM055_ADDRESS, BMM050_DIG_Z3_LSB);    // < trim z3 data
  dig_z4 = readS16_LE(BMM055_ADDRESS, BMM050_DIG_Z4_LSB);    // < trim z4 data 

  dig_xyz1 = read16_LE(BMM055_ADDRESS, BMM050_DIG_XYZ1_LSB); // < trim xyz1 data 
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
  write8(BMM055_ADDRESS, BMM055_CTRL_POWER, 0x00);
  delay(2); // wait until changes take effect
}

void bmm055::doSleepMode ()
{
  write8(BMM055_ADDRESS, BMM055_CTRL_POWER, 0x01);
  delay(2); // wait until changes take effect
}

void bmm055::doNormalMode ()
{
  write8(BMM055_ADDRESS, BMM055_CTRL_OP, 0x00);
  delay(2); // wait until changes take effect
}

void bmm055::doSoftReset ()
{
  write8(BMM055_ADDRESS, BMM055_CTRL_POWER, 0x83);
  delay(2); // wait until changes take effect
}

void bmm055::enableChannel(uint8_t channel, bool channel_state)
{
  controlInt2 = read8(BMM055_ADDRESS, BMM055_CTRL_INT2);
  switch (channel)
  {
    case 0: // Channel X
      if (channel_state)
        controlInt2 &= ~(1u << 3); // active-low, default is '0' enabled
      else
        controlInt2 |= (1u << 3);
      break;
    
    case 1: // Channel Y
      if (channel_state) 
        controlInt2 &= ~(1u << 4); // active-low, default is '0' enabled
      else
        controlInt2 |= (1u << 4);
      break;

    case 2: // Channel Z
      if (channel_state)
        controlInt2 &= ~(1u << 5); // active-low, default is '0' enabled
      else
        controlInt2 |= (1u << 5);
      break;
  }
  write8(BMM055_ADDRESS, BMM055_CTRL_INT2, controlInt2);
  delay(2); // wait until changes take effect
}


// REVISAR!
void bmm055::setSelfTest (bool enable)
{
  controlOp = read8(BMM055_ADDRESS, BMM055_CTRL_OP);

  if (enable)
    controlOp = 0x07; // Sleep Mode + self test = 1  //|= (1 << 0);  // self test bit = 0
  else
    controlOp &= ~(1 << 0); // self test bit = 0

  write8(BMM055_ADDRESS, BMM055_CTRL_OP, controlOp);
  delay(2); // wait until changes take effect
}


void bmm055::printSelfTest()
{
  uint8_t selfTestZ = read8(BMM055_ADDRESS, BMM055_DATAZ_LSB) & 1u;
  uint8_t selfTestY = read8(BMM055_ADDRESS, BMM055_DATAY_LSB) & 1u;
  uint8_t selfTestX = read8(BMM055_ADDRESS, BMM055_DATAX_LSB) & 1u;
  Serial.print("selfTestZ: 0x"); Serial.println(selfTestZ, HEX);
  Serial.print("selfTestY: 0x"); Serial.println(selfTestY, HEX);
  Serial.print("selfTestZ: 0x"); Serial.println(selfTestX, HEX);
  Serial.print("default is 0x"); Serial.println(0x1, HEX);
  Serial.println();
}


bool bmm055::getDRDY ()
{
  uint8_t DRDY = read8(BMM055_ADDRESS, BMM055_RHALL_LSB);
  return (DRDY & 1u);
}


void bmm055::getRawData ()
{
  // Burst Read! 9 bytes: rawDataX(2) + rawDataY(2) + rawDataZ(2) + rawRHall(2) + interruptStatusReg(1)
  uint8_t rawData[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

  Wire.beginTransmission((uint8_t)BMM055_ADDRESS);
  Wire.write((uint8_t)BMM055_MAG_DATA);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)BMM055_ADDRESS, (byte)9);

  for (int i=1; i<=9; ++i)
    rawData[i] = Wire.read();

  rawDataX = (rawData[1] << 8) | rawData[0];
  rawDataX >>= 3;
  rawDataY = (rawData[3] << 8) | rawData[2];
  rawDataY >>= 3;
  rawDataZ = (rawData[5] << 8) | rawData[4];
  rawDataZ >>= 1;
  rawRHall = (rawData[7] << 8) | rawData[6];
  rawRHall >>= 2;

  interruptStatusReg = rawData[8];

/*
  rawRHall = read16_LE(BMM055_ADDRESS, BMM055_RHALL_LSB) >> 2;
  rawDataZ = read16_LE(BMM055_ADDRESS, BMM055_DATAZ_LSB) >> 1;
  rawDataY = read16_LE(BMM055_ADDRESS, BMM055_DATAY_LSB) >> 3;
  rawDataX = read16_LE(BMM055_ADDRESS, BMM055_DATAX_LSB) >> 3;
  */
}

// Extracted by "bmm050_compensate_X_float" Bosch BMM150 API function
float bmm055::getCompensatedX ()
{
  float compensatedX = 0;

  if (rawDataX == BMM050_FLIP_OVERFLOW_ADCVAL) // overflow
    return BMM050_OVERFLOW_OUTPUT_FLOAT;
  else // no overflow
  {
    if (rawRHall != 0 && dig_xyz1 != 0)
      compensatedX = ((((float)dig_xyz1) * 16384.0 / rawRHall) - 16384.0);
    else
      return BMM050_OVERFLOW_OUTPUT_FLOAT;

    compensatedX = (((rawDataX * 
                    ((((((float)dig_xy2) * 
                     (compensatedX*compensatedX / 268435456.0) + 
                     compensatedX * ((float)dig_xy1)/ 16384.0)) + 256.0) * 
                   (((float)dig_x2) + 160.0))) / 8192.0) + (((float)dig_x1) * 8.0)) / 16.0; 
  }

  return compensatedX;
}


void bmm055::printTrimRegisters ()
{
  Serial.print("s8  dig_x1   : 0x"); Serial.println(dig_x1,   HEX);
  Serial.print("s8  dig_y1   : 0x"); Serial.println(dig_y1,   HEX);
  Serial.print("s8  dig_x2   : 0x"); Serial.println(dig_x2,   HEX);
  Serial.print("s8  dig_y2   : 0x"); Serial.println(dig_y2,   HEX);
  Serial.print("u8  dig_xy1  : 0x"); Serial.println(dig_xy1,  HEX);
  Serial.print("s8  dig_xy2  : 0x"); Serial.println(dig_xy2,  HEX);
  Serial.print("u16 dig_z1   : 0x"); Serial.println(dig_z1,   HEX);
  Serial.print("s16 dig_z2   : 0x"); Serial.println(dig_z2,   HEX);
  Serial.print("s16 dig_z3   : 0x"); Serial.println(dig_z3,   HEX);
  Serial.print("s16 dig_z4   : 0x"); Serial.println(dig_z4,   HEX);
  Serial.print("u16 dig_xyz1 : 0x"); Serial.println(dig_xyz1, HEX);
  Serial.println();
}




// Imprime todos los registros del magnetómetro sin desglose de información
void bmm055::printDefaultValues ()
{              //0x52         // 0x4A 
  for (byte reg=0x71; reg>=0x42; --reg)
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
  Serial.println();
}

void bmm055::printRepZ ()
{
  uint8_t repZ = read8(BMM055_ADDRESS, BMM055_CTRL_REP_Z);
  Serial.print("     repZ: 0x"); Serial.println(repZ, HEX);
  Serial.print("default is 0x"); Serial.println(0x00, HEX);
  Serial.println();
}

void bmm055::printRepXY ()
{
  uint8_t repXY = read8(BMM055_ADDRESS, BMM055_CTRL_REP_XY);
  Serial.print("    repXY: 0x"); Serial.println(repXY, HEX);
  Serial.print("default is 0x"); Serial.println(0x00, HEX);
  Serial.println();
}

void bmm055::printHighThreshold ()
{
  uint8_t highThreshold = read8(BMM055_ADDRESS, BMM055_CTRL_HIGH_THRESHOLD);
  Serial.print("highThreshold: 0x"); Serial.println(highThreshold, HEX);
  Serial.print("    default is 0x"); Serial.println(0x00, HEX);
  Serial.println();
}

void bmm055::printLowThreshold ()
{
  uint8_t lowThreshold = read8(BMM055_ADDRESS, BMM055_CTRL_LOW_THRESHOLD);
  Serial.print("lowThreshold: 0x"); Serial.println(lowThreshold, HEX);
  Serial.print("   default is 0x"); Serial.println(0x00, HEX);
  Serial.println();
}


// Control Interruption & Axis Register 2 Desglosado
void bmm055::printControlInt2 ()
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
  Serial.println();
}


// Control Interruption Register 1 Desglosado
void bmm055::printControlInt1 () 
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
  Serial.println();
}

// Control Operation Register Desglosado
void bmm055::printControlOp ()
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
  Serial.println();
}

// Control Power Register Desglosado
void bmm055::printPowerStatus ()
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
  Serial.println();
}

// Interrupt Status Register Desglosado
void bmm055::printInterruptStatus ()
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
  Serial.println();
}

// Imprime el valor raw de X, Y, Z y RHALL
void bmm055::printRawData ()
{
  Serial.print("rawRHall: 0x");
  Serial.println(rawRHall, HEX);
  
  Serial.print("rawDataZ: 0x");
  Serial.println(rawDataZ, HEX);

  Serial.print("rawDataY: 0x");
  Serial.println(rawDataY, HEX);

  Serial.print("rawDataX: 0x");
  Serial.println(rawDataX, HEX);
  Serial.println();
}


void bmm055::printChipID ()
{
  chipID = read8(BMM055_ADDRESS, BMM055_CHIP_ID);
  Serial.print("     Chip ID: 0x"); Serial.println(chipID, HEX);
  Serial.println("default is: 0x32");
  Serial.println();
}




