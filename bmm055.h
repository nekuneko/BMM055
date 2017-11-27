#ifndef __BMM055_H__
#define __BMM055_H__

#include <Arduino.h>

#define BMM055_ADDRESS 0x10

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

class bmm055 
{
public:
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

  bmm055();

  // Imprime todos los registros del magnetómetro sin desglose de información
  void printMagDefaultValues ();



  // ¡PROXIMO DÍA! IMPLEMENTAR LA FUNCION PARA VER EL ESTADO DEL CONTROL INT 2


  // Control Interruption Register 1 Desglosado
  void printMagControlINT1 ();

  // Control Operation Register Desglosado
  void printMagControlOP ();

  // Control Power Register Desglosado
  void printMagPowerStatus ();

  // Interrupt Status Register Desglosado
  void printMagInterruptStatus ();

  // Imprime el valor raw de X, Y, Z y RHALL
  void printMagRawData ();

  void printMagChipID ();


  void setSuspendMode ();

  void setSleepMode ();
};






#endif // __BMM055_H__
