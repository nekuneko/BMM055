#ifndef __BMM055_H__
#define __BMM055_H__

#include <Arduino.h>
#include "i2c_adafruit.h"

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

  BMM055_INT_STATUS           = 0x4A, // [RO]
  BMM055_RHALL_MSB            = 0x49, // [RO]
  BMM055_RHALL_LSB            = 0x48, // [RO]
  BMM055_DATAZ_MSB            = 0x47, // [RO]
  BMM055_DATAZ_LSB            = 0x46, // [RO]
  BMM055_DATAY_MSB            = 0x45, // [RO]
  BMM055_DATAY_LSB            = 0x44, // [RO]
  BMM055_DATAX_MSB            = 0x43, // [RO]
  BMM055_DATAX_LSB            = 0x42, // [RO]
  //BMM055_MAG_DATA           = 0x42, // [RO] 0x42 - 0x49 DATAX, DATAY, DATAZ, RHALL
    
  BMM055_CHIP_ID              = 0x40,  // [RO]
};

class bmm055 
{
public:
  uint8_t chipID                = 0;     // Addr 0x40 def val 0x32
  uint8_t controlRepZ           = 0;     // Addr 0x52 def val 0x00 -> 
  uint8_t controlRepXY          = 0;     // Addr 0x51 def val 0x00 ->
  uint8_t controlHighThreshold  = 0;     // Addr 0x50 def val 0x00 ->
  uint8_t controlLowThreshold   = 0;     // Addr 0x4F def val 0x00 ->
  uint8_t controlInt2           = 0;     // Addr 0x4E def val 0x07 -> 
  uint8_t controlInt1           = 0;     // Addr 0x4D def val 0x3f ->
  uint8_t controlOp             = 0;     // Addr 0x4C def val 0x06 -> Operation Mode, data rate 
  uint8_t controlPower          = 0;     // Addr 0x4B def val 0x01 -> Power control, soft reset
  uint8_t interruptStatusReg    = 0;     // Addr 0x4A def val 0x00 -> INTERRUPT_STATUS_  uint16_t rawRHall          
  uint16_t rawRHall             = 0;     // Addr 0x49 + 0x48
  uint16_t rawDataZ             = 0;     // Addr 0x47 + 0x46
  uint16_t rawDataY             = 0;     // Addr 0x45 + 0x44
  uint16_t rawDataX             = 0;     // Addr 0x43 + 0x42

// Constructor
  bmm055();


// Métodos modificadores
  void setRepZ          (uint8_t value);
  void setRepXY         (uint8_t value);
  void setHighThreshold (uint8_t value); 
  void setLowThreshold  (uint8_t value); 
  void setControlInt2   (uint8_t value);
  void setControlInt1   (uint8_t value);
  void setControlOp     (uint8_t value);

  void doSuspendMode ();
  void doSleepMode   ();
  void doSoftReset   ();


// Métodos observadores
  // Imprime todos los registros del magnetómetro sin desglose de información
  void printMagDefaultValues ();

  // Imprime el número de repeticiones en el eje Z
  void printMagRepZ ();

  // Imprime el número de repeticiones en los ejes X e Y
  void printMagRepXY ();

  // Imprime el valor de umbral máximo configurado
  void printMagHighThreshold ();

  // Imprime el valor de umbral mínimo configurado
  void printMagLowThreshold ();

  // Control Interruption & Axis Register 2 Desglosado
  void printMagControlInt2 ();

  // Control Interruption Register 1 Desglosado
  void printMagControlInt1 ();

  // Control Operation Register Desglosado
  void printMagControlOp ();

  // Control Power Register Desglosado
  void printMagPowerStatus ();

  // Interrupt Status Register Desglosado
  void printMagInterruptStatus ();

  // Imprime el valor raw de X, Y, Z y RHALL
  void printMagRawData ();

  void printMagChipID ();
};






#endif // __BMM055_H__
