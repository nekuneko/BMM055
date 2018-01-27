#ifndef __BMM055_H__
#define __BMM055_H__

#include "i2c_helper.h"

typedef int16_t s16;
typedef int32_t s32;

#define BMM055_ADDRESS 0x10
#define BMM055_CHIP_ID_VALUE 0x32
#define BMM050_INIT_VALUE 0

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


/********************************************/
/**\name OVERFLOW DEFINITIONS  */
/********************************************/
/* compensated output value returned if sensor had overflow */
#define BMM050_OVERFLOW_OUTPUT			-32768
#define BMM050_OVERFLOW_OUTPUT_S32		((s32)(-2147483647-1))
#define BMM050_OVERFLOW_OUTPUT_FLOAT	0.0f
#define BMM050_FLIP_OVERFLOW_ADCVAL		-4096
#define BMM050_HALL_OVERFLOW_ADCVAL	-16384

/********************************************/
/**\name DATA RATE DEFINITIONS  */
/********************************************/
/* Data Rates */
#define BMM050_DR_10HZ                     (0)
#define BMM050_DR_02HZ                     (1)
#define BMM050_DR_06HZ                     (2)
#define BMM050_DR_08HZ                     (3)
#define BMM050_DR_15HZ                     (4)
#define BMM050_DR_20HZ                     (5)
#define BMM050_DR_25HZ                     (6)
#define BMM050_DR_30HZ                     (7)

#define BMM050_DATA_RATE_10HZ        (0x00)
#define BMM050_DATA_RATE_02HZ        (0x01)
#define BMM050_DATA_RATE_06HZ        (0x02)
#define BMM050_DATA_RATE_08HZ        (0x03)
#define BMM050_DATA_RATE_15HZ        (0x04)
#define BMM050_DATA_RATE_20HZ        (0x05)
#define BMM050_DATA_RATE_25HZ        (0x06)
#define BMM050_DATA_RATE_30HZ (0x07)

/********************************************/
/**\name PRESET MODE DEFINITIONS  */
/********************************************/
#define BMM050_PRESETMODE_LOWPOWER                  (1)
#define BMM050_PRESETMODE_REGULAR                   (2)
#define BMM050_PRESETMODE_HIGHACCURACY              (3)
#define BMM050_PRESETMODE_ENHANCED                  (4)

/* PRESET MODES - DATA RATES */
#define BMM050_LOWPOWER_DR                       (BMM050_DR_10HZ)
#define BMM050_REGULAR_DR                        (BMM050_DR_10HZ)
#define BMM050_HIGHACCURACY_DR                   (BMM050_DR_20HZ)
#define BMM050_ENHANCED_DR                       (BMM050_DR_10HZ)

/* PRESET MODES - REPETITIONS-XY RATES */
#define BMM050_LOWPOWER_REPXY                     (1)
#define BMM050_REGULAR_REPXY                      (4)
#define BMM050_HIGHACCURACY_REPXY                (23)
#define BMM050_ENHANCED_REPXY                     (7)

/* PRESET MODES - REPETITIONS-Z RATES */
#define BMM050_LOWPOWER_REPZ                      (2)
#define BMM050_REGULAR_REPZ                      (14)
#define BMM050_HIGHACCURACY_REPZ                 (82)
#define BMM050_ENHANCED_REPZ                     (26)





/********************************************/
/**\name BIT SHIFTING DEFINITIONS  */
/********************************************/
/*Shifting Constants*/
#define BMM050_SHIFT_BIT_POSITION_BY_01_BIT     (1)
#define BMM050_SHIFT_BIT_POSITION_BY_02_BITS    (2)
#define BMM050_SHIFT_BIT_POSITION_BY_03_BITS    (3)
#define BMM050_SHIFT_BIT_POSITION_BY_05_BITS    (5)
#define BMM050_SHIFT_BIT_POSITION_BY_06_BITS    (6)
#define BMM050_SHIFT_BIT_POSITION_BY_07_BITS    (7)
#define BMM050_SHIFT_BIT_POSITION_BY_08_BITS    (8)
#define BMM050_SHIFT_BIT_POSITION_BY_09_BITS    (9)
#define BMM050_SHIFT_BIT_POSITION_BY_12_BITS    (12)
#define BMM050_SHIFT_BIT_POSITION_BY_13_BITS    (13)
#define BMM050_SHIFT_BIT_POSITION_BY_16_BITS    (16)
#define BMM050_SHIFT_BIT_POSITION_BY_14_BITS    (14)
#define BMM050_SHIFT_BIT_POSITION_BY_15_BITS    (15)



class bmm055 
{
public:     
  uint16_t rawRHall = 0;     // Addr 0x49 + 0x48
  int16_t  rawDataZ = 0;     // Addr 0x47 + 0x46
  int16_t  rawDataY = 0;     // Addr 0x45 + 0x44
  int16_t  rawDataX = 0;     // Addr 0x43 + 0x42

  float datax;/**<mag compensated X  data*/
  float datay;/**<mag compensated Y  data*/
  float dataz;/**<mag compensated Z  data*/
  uint16_t resistance;/**<mag R  data*/
  bool data_ready;/**<mag data ready status*/

  // Trim Registers
  int8_t dig_x1;/**< trim x1 data */
  int8_t dig_y1;/**< trim y1 data */
  int8_t dig_x2;/**< trim x2 data */
  int8_t dig_y2;/**< trim y2 data */

  uint16_t dig_z1;/**< trim z1 data */
  int16_t  dig_z2;/**< trim z2 data */
  int16_t  dig_z3;/**< trim z3 data */
  int16_t  dig_z4;/**< trim z4 data */

  uint8_t  dig_xy1;/**< trim xy1 data */
  int8_t   dig_xy2;/**< trim xy2 data */
  uint16_t dig_xyz1;/**< trim xyz1 data */


// Constructor
  bmm055();


  bool init(uint8_t BMM050_PRESETMODE = BMM050_PRESETMODE_REGULAR);
  void init_trim_registers ();

// Métodos modificadores
  void setPresetMode    (uint8_t value);
  void setDataRate      (uint8_t value);
  void setRepZ          (uint8_t value);
  void setRepXY         (uint8_t value);
  void setHighThreshold (uint8_t value); 
  void setLowThreshold  (uint8_t value); 

  void setSuspendMode ();
  void setSleepMode   ();
  void setSoftReset   (); // REVISAR
  void setNormalMode  ();

  // @brief This API used to set the self test of the sensor in the register 0x4C bit 0
  // @param set : The value of selftest
  // @note write true to start self test
  void setSelfTest (bool set);

  void enableChannel(uint8_t channel, bool channel_state);


  uint8_t getChipID();
  bool getDRDY ();


  void getRawData ();
  float getHeading();

  float getCompensatedX ();
  float getCompensatedY ();
  float getCompensatedZ ();

  void updateMagData();

// Métodos observadores
  float  x () { return datax; }
  float  y () { return datay; }
  float  z () { return dataz; }
  uint16_t r () { return resistance; }


  uint8_t getPowerStatus ();
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
