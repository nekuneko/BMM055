
#include "i2c_adafruit.h"
//#include "BMM055.h"

#define BMM055_ADDR 0x10
#define MAG_ADDR 0x10


// BMM055 Registers
enum 
{
  BMM055_CTRL_REP_Z           = 0x52, // [RW]
  BMM055_CTRL_REP_XY          = 0x51, // [RW]
  BMM055_CTRL_HIGH_THRESHOLD  = 0x50, // [RW]
  BMM055_CTRL_LOW_THRESHOLD   = 0x4F, // [RW]
  BMM055_CTRL_INT2            = 0x4E, // [RW]
  BMM055_CTRL_INT1            = 0x4D, // [RW]
  BMM055_CTRL_OP              = 0x4C, // [RW]
  BMM055_CTRL_POWER           = 0x4B, // [RW] accesible in suspend mode

  BMM055_INT_STATUS = 0x4A, // [RO]
  BMM055_RHALL_MSB  = 0x49, // [RO]
  BMM055_RHALL_LSB  = 0x48, // [RO]
  BMM055_DATAZ_MSB  = 0x47, // [RO]
  BMM055_DATAZ_LSB  = 0x46, // [RO]
  BMM055_DATAY_MSB  = 0x45, // [RO]
  BMM055_DATAY_LSB  = 0x44, // [RO]
  BMM055_DATAX_MSB  = 0x43, // [RO]
  BMM055_DATAX_LSB  = 0x42, // [RO]
  //BMM055_MAG_DATA   = 0x42, // [RO] 0x42 - 0x49 DATAX, DATAY, DATAZ, RHALL
    
  BMM055_CHIP_ID   = 0x40,  // [RO]
};


uint8_t repZ  = 0;                // Reg Addr 0x52 def val 0x00 -> 
uint8_t repXY = 0;                // Reg Addr 0x51 def val 0x00 ->
uint8_t highThreshold = 0;        // Reg Addr 0x50 def val 0x00 ->
uint8_t lowThreshold  = 0;        // Reg Addr 0x4F def val 0x00 ->
uint8_t controlINT2 = 0;          // Reg Addr 0x4E def val 0x07 -> 
uint8_t controlINT1 = 0;          // Reg Addr 0x4D def val 0x3f ->
uint8_t controlOP = 0;            // Reg Addr 0x4C def val 0x06 -> Operation Mode, data rate 
uint8_t controlPower = 0;         // Reg Addr 0x4B def val 0x01 -> Power control, soft reset
uint8_t interruptStatusReg = 0;   // Reg Addr 0x4A def val 0x00 -> INTERRUPT_STATUS_REG
uint16_t rawRHall = 0;            // Reg Addr 0x49 + 0x48
uint16_t rawDataZ = 0;            // Reg Addr 0x47 + 0x46
uint16_t rawDataY = 0;            // Reg Addr 0x45 + 0x44
uint16_t rawDataX = 0;            // Reg Addr 0x43 + 0x42
uint8_t chipID = 0;              // Reg Addr 0x40 def val 0x32




// Imprime todos los registros del magnetómetro sin desglose de información
void printMagDefaultValues ()
{ 
  for (byte reg=0x52; reg>=0x4A; --reg)
  {
    Serial.print("REG 0x");
    Serial.print(reg, HEX);
    Serial.print(": 0x");
    Serial.println(read8(BMM055_ADDR, reg), HEX);
  }
  
  // Chip ID
  Serial.print("REG 0x");
  Serial.print(BMM055_CHIP_ID, HEX);
  Serial.print(": 0x");
  Serial.println(read8(BMM055_ADDR, BMM055_CHIP_ID), HEX);
  
}



// ¡PROXIMO DÍA! IMPLEMENTAR LA FUNCION PARA VER EL ESTADO DEL CONTROL INT 2


// Control Interruption Register 1 Desglosado
void printMagControlINT1 ()
{
  controlINT1 = read8(BMM055_ADDR, BMM055_CTRL_INT1); 
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
void printMagControlOP ()
{
  controlOP = read8(MAG_ADDR, BMM055_CTRL_OP); 
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
void printMagPowerStatus ()
{
  controlPower = read8(BMM055_ADDR, BMM055_CTRL_POWER);
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
void printMagInterruptStatus ()
{
  interruptStatusReg = read8(MAG_ADDR, BMM055_INT_STATUS);
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
void printMagRawData ()
{
  rawRHall = read16_LE(MAG_ADDR, 0x48) >> 2;
  rawDataZ = read16_LE(MAG_ADDR, 0x46) >> 1;
  rawDataY = read16_LE(MAG_ADDR, 0x44) >> 3;
  rawDataX = read16_LE(MAG_ADDR, 0x42) >> 3;

  Serial.print("rawRHall: 0x");
  Serial.println(rawRHall, HEX);
  
  Serial.print("rawDataZ: 0x");
  Serial.println(rawDataZ, HEX);

  Serial.print("rawDataZ: 0x");
  Serial.println(rawDataZ, HEX);

  Serial.print("rawDataZ: 0x");
  Serial.println(rawDataZ, HEX);
}


void printMagChipID ()
{
  chipID = read8(BMM055_ADDR, BMM055_CHIP_ID);
  Serial.print("Chip ID: 0x");
  Serial.println(chipID, HEX);
}

void setup() 
{
  i2c_init();
  Serial.begin(115200);
  while (!Serial); // Esperar a que se abra el serial
  Serial.println("BMM055 NEKU test");

  
  // Suspend Mode
  Serial.println("Suspend Mode");
  write8(BMM055_ADDR, BMM055_CTRL_POWER, 0x0);
  delay(2); // asegurar escritura
  //printMagPowerStatus();
  //Serial.println();
  //delay(1000);
  
  // Sleep Mode
  Serial.println("Sleep Mode");
  write8(BMM055_ADDR, BMM055_CTRL_POWER, 0x1);
  delay(2); // asegurar escritura
  //printMagPowerStatus();
  //Serial.println();
  //delay(1000);

  // OpMode Normal
  printMagControlINT1();
 

  
  

  /*
  // Poner en normal operation mode
  controlOP &= ~(1u << 2);
  controlOP &= ~(1u << 1); 
  write8(BMM055_ADDR, BMM055_CTRL_OP, controlOP); 
  
  printControlOP();
  //printMagInterruptStatus();
  //printMagDefaultValues();
  */
  
}

void loop() 
{
 // printRawMagData();
  //Serial.println();
}
