#ifndef __BMM055_H__
#define __BMM055_H__

#include <Arduino.h>
#include "i2c_helper.h"

#define BMM055_ADDRESS 0x10

#define BMM055_CHANNEL_Z (2u)
#define BMM055_CHANNEL_Y (1u)
#define BMM055_CHANNEL_X (0u)

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
  BMM055_MAG_DATA             = 0x42, // [RO] 0x42 - 0x49 DATAX, DATAY, DATAZ, RHALL
    
  BMM055_CHIP_ID              = 0x40,  // [RO]
};


/* Trim Extended Registers */
#define BMM050_DIG_X1                      (0x5D)
#define BMM050_DIG_Y1                      (0x5E)
#define BMM050_DIG_Z4_LSB                  (0x62)
#define BMM050_DIG_Z4_MSB                  (0x63)
#define BMM050_DIG_X2                      (0x64)
#define BMM050_DIG_Y2                      (0x65)
#define BMM050_DIG_Z2_LSB                  (0x68)
#define BMM050_DIG_Z2_MSB                  (0x69)
#define BMM050_DIG_Z1_LSB                  (0x6A)
#define BMM050_DIG_Z1_MSB                  (0x6B)
#define BMM050_DIG_XYZ1_LSB                (0x6C)
#define BMM050_DIG_XYZ1_MSB                (0x6D)
#define BMM050_DIG_Z3_LSB                  (0x6E)
#define BMM050_DIG_Z3_MSB                  (0x6F)
#define BMM050_DIG_XY2                     (0x70)
#define BMM050_DIG_XY1                     (0x71)


/* compensated output value returned if sensor had overflow */
#define BMM050_OVERFLOW_OUTPUT      -32768
#define BMM050_OVERFLOW_OUTPUT_S32    ((s32)(-2147483647-1))
#define BMM050_OVERFLOW_OUTPUT_FLOAT  0.0f
#define BMM050_FLIP_OVERFLOW_ADCVAL   -4096
#define BMM050_HALL_OVERFLOW_ADCVAL   -16384




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

  float datax;/**<mag compensated X  data*/
  float datay;/**<mag compensated Y  data*/
  float  dataz;/**<mag compensated Z  data*/
  uint16_t resistance;/**<mag R  data*/
  uint8_t data_ready;/**<mag data ready status*/

  // Trim Registers
  int8_t dig_x1;/**< trim x1 data */
  int8_t dig_y1;/**< trim y1 data */

  int8_t dig_x2;/**< trim x2 data */
  int8_t dig_y2;/**< trim y2 data */

  uint16_t dig_z1;/**< trim z1 data */
  int16_t  dig_z2;/**< trim z2 data */
  int16_t  dig_z3;/**< trim z3 data */
  int16_t  dig_z4;/**< trim z4 data */

  uint8_t dig_xy1;/**< trim xy1 data */
  int8_t  dig_xy2;/**< trim xy2 data */

  uint16_t dig_xyz1;/**< trim xyz1 data */


// Constructor
  bmm055();

  init();
  void init_trim_registers ();

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
  void doNormalMode  ();

  // @brief This API used to set the self test of the sensor in the register 0x4C bit 0
  // @param set : The value of selftest
  // @note write true to start self test
  void setSelfTest (bool set);

  void enableChannel(uint8_t channel, bool channel_state);

  bool getDRDY ();
// Métodos observadores

  void getRawData ();

  float getCompensatedX ();
  //float getCompensatedY ();


  void printTrimRegisters ();

  // Imprime todos los registros del magnetómetro sin desglose de información
  void printDefaultValues ();

  void printSelfTest();

  // Imprime el número de repeticiones en el eje Z
  void printRepZ ();

  // Imprime el número de repeticiones en los ejes X e Y
  void printRepXY ();

  // Imprime el valor de umbral máximo configurado
  void printHighThreshold ();

  // Imprime el valor de umbral mínimo configurado
  void printLowThreshold ();

  // Control Interruption & Axis Register 2 Desglosado
  void printControlInt2 ();

  // Control Interruption Register 1 Desglosado
  void printControlInt1 ();

  // Control Operation Register Desglosado
  void printControlOp ();

  // Control Power Register Desglosado
  void printPowerStatus ();

  // Interrupt Status Register Desglosado
  void printInterruptStatus ();

  // Imprime el valor raw de X, Y, Z y RHALL
  void printRawData ();

  void printChipID ();
};






#endif // __BMM055_H__
