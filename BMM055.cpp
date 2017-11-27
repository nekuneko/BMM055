#include "BMM055.h"

// -- Defined in MultiWii.cpp
imu_t imu;
flags_struct_t f;
global_conf_t global_conf;
uint32_t currentTime = 0;

// -- Defined in Sensors.cpp --
#if !defined(MAG_ORIENTATION) 
  #define MAG_ORIENTATION(X, Y, Z)  {imu.magADC[ROLL]  = X; imu.magADC[PITCH]  = Y; imu.magADC[YAW]  = Z;}
#endif

// ************************************************************************************************************
// I2C Compass common function
// ************************************************************************************************************
#if MAG
static float   magGain[3] = {1.0,1.0,1.0};  // gain for each axis, populated at sensor init
static uint8_t magInit = 0;

uint8_t Mag_getADC() { // return 1 when news values are available, 0 otherwise
  static uint32_t t,tCal = 0;
  static int16_t magZeroTempMin[3];
  static int16_t magZeroTempMax[3];
  uint8_t axis;
  if ( currentTime < t ) return 0; //each read is spaced by 100ms
  t = currentTime + 100000;
  TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
  Device_Mag_getADC();
  imu.magADC[ROLL]  = imu.magADC[ROLL]  * magGain[ROLL];
  imu.magADC[PITCH] = imu.magADC[PITCH] * magGain[PITCH];
  imu.magADC[YAW]   = imu.magADC[YAW]   * magGain[YAW];
  if (f.CALIBRATE_MAG) {
    tCal = t;
    for(axis=0;axis<3;axis++) {
      global_conf.magZero[axis] = 0;
      magZeroTempMin[axis] = imu.magADC[axis];
      magZeroTempMax[axis] = imu.magADC[axis];
    }
    f.CALIBRATE_MAG = 0;
  }
  if (magInit) { // we apply offset only once mag calibration is done
    imu.magADC[ROLL]  -= global_conf.magZero[ROLL];
    imu.magADC[PITCH] -= global_conf.magZero[PITCH];
    imu.magADC[YAW]   -= global_conf.magZero[YAW];
  }
 
  if (tCal != 0) {
    if ((t - tCal) < 30000000) { // 30s: you have 30s to turn the multi in all directions
/////      LEDPIN_TOGGLE;
      for(axis=0;axis<3;axis++) {
        if (imu.magADC[axis] < magZeroTempMin[axis]) magZeroTempMin[axis] = imu.magADC[axis];
        if (imu.magADC[axis] > magZeroTempMax[axis]) magZeroTempMax[axis] = imu.magADC[axis];
      }
    } else {
      tCal = 0;
      for(axis=0;axis<3;axis++)
        global_conf.magZero[axis] = (magZeroTempMin[axis] + magZeroTempMax[axis])>>1;
 ////     writeGlobalSet(1); // defined in EEPROM.cpp
    }
  } else {
    #if defined(SENSORS_TILT_45DEG_LEFT)
      int16_t temp = ((imu.magADC[PITCH] - imu.magADC[ROLL] )*7)/10;
      imu.magADC[ROLL] = ((imu.magADC[ROLL]  + imu.magADC[PITCH])*7)/10;
      imu.magADC[PITCH] = temp;
    #endif
    #if defined(SENSORS_TILT_45DEG_RIGHT)
      int16_t temp = ((imu.magADC[PITCH] + imu.magADC[ROLL] )*7)/10;
      imu.magADC[ROLL] = ((imu.magADC[ROLL]  - imu.magADC[PITCH])*7)/10;
      imu.magADC[PITCH] = temp;
    #endif
  }
  return 1;
}
#endif

/********* 
  >EL CÓDIGO DE ESTE DOCUMENTO VA EN EL FICHERO "Sensors.cpp", DE MULTIWII
  
  >EN EL FICHERO "def.h"
  SUSTITUIR DENTRO DEL FICHERO "def.h", EN LA SECCIÓN  "Sensor Type definitions"
  
    #if defined(HMC5883) || defined(HMC5843) || defined(AK8975) || defined(MAG3110)
    #define MAG 1

  POR
  
    #if defined(HMC5883) || defined(HMC5843) || defined(AK8975) || defined(MAG3110)|| defined(BMM055)
    #define MAG 1

  O SIMPLEMENTE AÑADIR AL FINAL "|| defined(BMM055)" SIN LAS COMILLAS
  
  >EN EL FICHERO "Config.h" AÑADIR EN EN LA SECCIÓN "independent sensors", CONCRETAMENTE EN LA SUBSECCIÓN "I2C magnetometer"

    #define BMM085
**********/


// -- COPIE EL CÓDIGO A PARTIR DE AQUÍ JUNTO A LOS DEMÁS MAGNETÓMETROS DEFINIDOS EN "Sensors.cpp"
// ************************************************************************************************************
// I2C Magnetometer BMM055
// ************************************************************************************************************
// Resolution: 
//  13 bits for X/Y axis
//  15 bits for Z   axis
//  14 bits for RHALL                      
// ************************************************************************************************************
#if defined(BMM055)
#define BMM055_ADDRESS 0x10 //0x11 0x12 0x13. See D.S. BMX055 Table 64 I2C address. 

/**\name  TRIM REGISTERS      */
/********************************************/
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
#define BMM050_DIG_XY1 (0x71)

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

/********************************************/
/**\name OVERFLOW DEFINITIONS  */
/********************************************/
/* compensated output value returned if sensor had overflow */
#define BMM050_OVERFLOW_OUTPUT      -32768
#define BMM050_OVERFLOW_OUTPUT_S32    ((s32)(-2147483647-1))
#define BMM050_OVERFLOW_OUTPUT_FLOAT  0.0f
#define BMM050_FLIP_OVERFLOW_ADCVAL   -4096
#define BMM050_HALL_OVERFLOW_ADCVAL -16384


/********************************************/
/**\name SELF TEST DEFINITIONS  */
/********************************************/
#define BMM050_ADVANCED_SELFTEST_OFF            (0)
#define BMM050_ADVANCED_SELFTEST_NEGATIVE       (2)
#define BMM050_ADVANCED_SELFTEST_POSITIVE       (3)

#define BMM050_NEGATIVE_SATURATION_Z            (-32767)
#define BMM050_POSITIVE_SATURATION_Z (32767)

// BMM055 REGISTERS
enum
{
  BMM055_REGISTER_CHIP_ID   = 0x40, // [RO]

  BMM055_REGISTER_MAG_DATA  = 0x42, // [RO] 0x42 - 0x49 DATAX, DATAY, DATAZ, RHALL
  BMM055_REGISTER_DATAX_LSB = 0x42, // [RO]
  BMM055_REGISTER_DATAX_MSB = 0x43, // [RO]
  BMM055_REGISTER_DATAY_LSB = 0x44, // [RO]
  BMM055_REGISTER_DATAY_MSB = 0x45, // [RO]
  BMM055_REGISTER_DATAZ_LSB = 0x46, // [RO]
  BMM055_REGISTER_DATAZ_MSB = 0x47, // [RO]
  BMM055_REGISTER_RHALL_LSB = 0x48, // [RO]
  BMM055_REGISTER_RHALL_MSB = 0x49, // [RO]

  BMM055_REGISTER_STATE_INT   = 0x4A, // [RO]

  BMM055_REGISTER_CTRL_POWER  = 0x4B, // [RW]
  BMM055_REGISTER_CTRL_OP     = 0x4C, // [RW]
  BMM055_REGISTER_CTRL_INT_1  = 0x4D, // [RW]
  BMM055_REGISTER_CTRL_INT_2  = 0x4E, // [RW]

  BMM055_REGISTER_LOW_THRESHOLD   = 0x4F, // [RW]
  BMM055_REGISTER_HIGH_THRESHOLD  = 0x50, // [RW]

  BMM055_REGISTER_CTRL_REP_XY     = 0x51, // [RW]
  BMM055_REGISTER_CTRL_REP_Z      = 0x52, // [RW]

  // Note:
  // RW - Read/Write
  // RO - Read  Only
  // WO - Write Only
};

struct bmm055_t 
{
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
};

static struct bmm055_t *bmm055_calib_data;


void bmm055_init_trim_registers ()
{
  bmm055_calib_data->dig_x1   = i2c_readReg(BMM055_ADDRESS, BMM050_DIG_X1);
  bmm055_calib_data->dig_y1   = i2c_readReg(BMM055_ADDRESS, BMM050_DIG_Y1);
  bmm055_calib_data->dig_x2   = i2c_readReg(BMM055_ADDRESS, BMM050_DIG_X2);
  bmm055_calib_data->dig_y2   = i2c_readReg(BMM055_ADDRESS, BMM050_DIG_Y2);
  bmm055_calib_data->dig_xy1  = i2c_readReg(BMM055_ADDRESS, BMM050_DIG_XY1);
  bmm055_calib_data->dig_xy2  = i2c_readReg(BMM055_ADDRESS, BMM050_DIG_XY2);

 /* shorts can not be recast into (u8*)
  * due to possible mix up between trim data
  * arrangement and memory arrangement */

  bmm055_calib_data->dig_z1 =  read16_LE(BMM055_ADDRESS, BMM050_DIG_Z1_LSB);
  bmm055_calib_data->dig_z2 = readS16_LE(BMM055_ADDRESS, BMM050_DIG_Z2_LSB);
  bmm055_calib_data->dig_z3 = readS16_LE(BMM055_ADDRESS, BMM050_DIG_Z3_LSB);
  bmm055_calib_data->dig_z4 = readS16_LE(BMM055_ADDRESS, BMM050_DIG_Z4_LSB);

  bmm055_calib_data->dig_xyz1 = read16_LE(BMM055_ADDRESS, BMM050_DIG_XYZ1_LSB);
}


int16_t bmm055_compensate_X (int16_t mag_data_x, uint16_t data_r)
{
  int16_t inter_retval = 0;
  
  /* no overflow */
  if (mag_data_x != BMM050_FLIP_OVERFLOW_ADCVAL) 
  {
    if ((data_r != 0) && (bmm055_calib_data->dig_xyz1 != 0)) 
    {
      inter_retval = ((int16_t)(((uint16_t)
      ((((int32_t)bmm055_calib_data->dig_xyz1) << BMM050_SHIFT_BIT_POSITION_BY_14_BITS)/ (data_r != 0 ? data_r : bmm055_calib_data->dig_xyz1))) - ((uint16_t)0x4000)));
    } 
    else 
    {
      inter_retval = BMM050_OVERFLOW_OUTPUT;
      return inter_retval;
    }
    inter_retval = ((int16_t)((((int32_t)mag_data_x) * ((((((((int32_t)bmm055_calib_data->dig_xy2) * ((((int32_t)inter_retval) * ((int32_t)inter_retval)) >> BMM050_SHIFT_BIT_POSITION_BY_07_BITS)) +
                   (((int32_t)inter_retval) * ((int32_t)(((int16_t)bmm055_calib_data->dig_xy1) << BMM050_SHIFT_BIT_POSITION_BY_07_BITS)))) >> BMM050_SHIFT_BIT_POSITION_BY_09_BITS) +
                   ((int32_t)0x100000)) * ((int32_t)(((int16_t)bmm055_calib_data->dig_x2) +
                   ((int16_t)0xA0)))) >> BMM050_SHIFT_BIT_POSITION_BY_12_BITS)) >> BMM050_SHIFT_BIT_POSITION_BY_13_BITS)) + 
                   (((int16_t)bmm055_calib_data->dig_x1) << BMM050_SHIFT_BIT_POSITION_BY_03_BITS);
  } 
  else 
  {
    /* overflow */
    inter_retval = BMM050_OVERFLOW_OUTPUT;
  }

  return inter_retval;
}

int16_t bmm055_compensate_Y (int16_t mag_data_y, uint16_t data_r)
{
  int16_t inter_retval = 0;
  
  /* no overflow */
  if (mag_data_y != BMM050_FLIP_OVERFLOW_ADCVAL) 
  {
    if ((data_r != 0) && (bmm055_calib_data->dig_xyz1 != 0)) 
    {
      inter_retval = ((int16_t)(((uint16_t)((( (int32_t)bmm055_calib_data->dig_xyz1) << BMM050_SHIFT_BIT_POSITION_BY_14_BITS)/(data_r != 0 ? data_r : bmm055_calib_data->dig_xyz1))) - ((uint16_t)0x4000)));
    } 
    else 
    {
      inter_retval = BMM050_OVERFLOW_OUTPUT;
      return inter_retval;
    }
    inter_retval = ((int16_t)((((int32_t)mag_data_y) * ((((((((int32_t) bmm055_calib_data->dig_xy2) * ((((int32_t) inter_retval) * ((int32_t)inter_retval)) >> BMM050_SHIFT_BIT_POSITION_BY_07_BITS)) + 
                   (((int32_t)inter_retval) * ((int32_t)(((int16_t)bmm055_calib_data->dig_xy1) << BMM050_SHIFT_BIT_POSITION_BY_07_BITS)))) >> BMM050_SHIFT_BIT_POSITION_BY_09_BITS) + 
                   ((int32_t)0x100000)) * ((int32_t)(((int16_t)bmm055_calib_data->dig_y2) + 
                   ((int16_t)0xA0)))) >> BMM050_SHIFT_BIT_POSITION_BY_12_BITS)) >> BMM050_SHIFT_BIT_POSITION_BY_13_BITS)) +
                   (((int16_t)bmm055_calib_data->dig_y1) << BMM050_SHIFT_BIT_POSITION_BY_03_BITS);
  } else 
  {
    /* overflow */
    inter_retval = BMM050_OVERFLOW_OUTPUT;
  }

  return inter_retval;
}


int16_t bmm055_compensate_Z (int16_t mag_data_z, uint16_t data_r)
{
  int32_t retval = 0;

  /* no overflow */
  if ((mag_data_z != BMM050_HALL_OVERFLOW_ADCVAL)) 
  {
    if ((bmm055_calib_data->dig_z2 != 0) && (bmm055_calib_data->dig_z1 != 0) && 
        (data_r != 0) && (bmm055_calib_data->dig_xyz1 != 0)) 
    {
      retval = (((((int32_t)(mag_data_z - bmm055_calib_data->dig_z4)) << BMM050_SHIFT_BIT_POSITION_BY_15_BITS) - ((((int32_t)bmm055_calib_data->dig_z3) * ((int32_t)(((int16_t)data_r) - ((int16_t) bmm055_calib_data->dig_xyz1)))) >> BMM050_SHIFT_BIT_POSITION_BY_02_BITS))/(bmm055_calib_data->dig_z2 + 
               ((int16_t)(((((int32_t) bmm055_calib_data->dig_z1) * ((((int16_t)data_r) << BMM050_SHIFT_BIT_POSITION_BY_01_BIT))) +
               (1 << BMM050_SHIFT_BIT_POSITION_BY_15_BITS)) >> BMM050_SHIFT_BIT_POSITION_BY_16_BITS))));
    } 
    else 
    {
      retval = BMM050_OVERFLOW_OUTPUT;
      return retval;
    }
    /* saturate result to +/- 2 microTesla */
    if (retval > BMM050_POSITIVE_SATURATION_Z) 
    {
      retval =  BMM050_POSITIVE_SATURATION_Z;
    } else 
    {
      if (retval < BMM050_NEGATIVE_SATURATION_Z)
        retval = BMM050_NEGATIVE_SATURATION_Z;
    }
  } 
  else 
  {
    /* overflow */
    retval = BMM050_OVERFLOW_OUTPUT;
  }
  
  return (int16_t)retval;
}


void Mag_init() 
{ 
  // 1. Suspend Mode to Sleep Mode
  i2c_writeReg(BMM055_ADDRESS, BMM055_REGISTER_CTRL_POWER, 0x01);   // 0x01 = 0 000 0 0 1b    default: soft_reset off, spi  off,  Sleep Mode (power on device)

  // 1.1 Read Trim Values
  bmm055_init_trim_registers();

  // 2. Set the number of measurement repetitions for each channel, select one of this two modes
  // 2.1 ENHACED REGULAR PRESET
    //i2c_writeReg(BMM055_ADDRESS, BMM055_REGISTER_CTRL_REP_XY, 0x07);  // 0x07 = 0000 0111b    nXY = 15 repetitions in measurement
    //i2c_writeReg(BMM055_ADDRESS, BMM055_REGISTER_CTRL_REP_Z,  0x1A);  // 0x1A = 0001 1010b    nZ  = 27 repetitions in measurement
  // 2.2 HIGH ACCURACY PRESET
  i2c_writeReg(BMM055_ADDRESS, BMM055_REGISTER_CTRL_REP_XY, 0x17);  // 0x17 = 0001 0111b      nXY = 47 repetitions in measurement
  i2c_writeReg(BMM055_ADDRESS, BMM055_REGISTER_CTRL_REP_Z,  0x52);  // 0x52 = 0101 0010b      nZ  = 83 repetitions in measurement

  // 3. Set LOW & HIGH THRESHOLDs - Here we won't use them, but it must be the initial value
  i2c_writeReg(BMM055_ADDRESS, BMM055_REGISTER_LOW_THRESHOLD,  0x00); // 0x00   default: not neccesary for this application
  i2c_writeReg(BMM055_ADDRESS, BMM055_REGISTER_HIGH_THRESHOLD, 0x00); // 0x00   default: not neccesary for this application

  // 4. Enable axis x/y/z and rhall measurement
  i2c_writeReg(BMM055_ADDRESS, BMM055_REGISTER_CTRL_INT_2, 0x07); // 0x07 = 0000 0111b    default: enable x/y/z axis and resistance measurement

  // 5. Disable all hardware interruptions
  i2c_writeReg(BMM055_ADDRESS, BMM055_REGISTER_CTRL_INT_1, 0x3F); // 0x3F = 0011 1111b    default: all interrupts disabled

  // 6. Sleep Mode to Active Mode and set ODR, select one of this two modes
  // 6.1 ENHACED REGULAR PRESET
    //i2c_writeReg(BMM055_ADDRESS, BMM055_REGISTER_CTRL_OP, 0x00);  // 0x00 = 00 000 00 0b    no advanced self-test, ODR 10 Hz (default), OpMode Normal, no self-test
  // 6.2 HIGH ACCURACY PRESET
  i2c_writeReg(BMM055_ADDRESS, BMM055_REGISTER_CTRL_OP, 0x28);  // 0x28 = 00 101 00 0b    no advanced self-test, ODR 20 Hz, OpMode Normal, no self-test

  // Measuring Started in BMM055!
  
  // Indicates that magnetometer is initializated, this variable is defined in Compass common function in Sensors.cpp
  magInit = 1;

#ifdef SERIAL_DEBUG_MAG
  Serial.println("Magnetómetro Iniciado");
  Serial.println();
#endif // SERIAL_DEBUG_MAG
}


void Device_Mag_getADC() 
{ 
  /*
   MAGrawADC[0] = DATAX_LSB (0x42)
   MAGrawADC[1] = DATAX_MSB (0x43)
   MAGrawADC[2] = DATAY_LSB (0x44)
   MAGrawADC[3] = DATAY_MSB (0x45)
   MAGrawADC[4] = DATAZ_LSB (0x46)
   MAGrawADC[5] = DATAZ_MSB (0x47)
   MAGrawADC[6] = RHALL_LSB (0x48)
   MAGrawADC[7] = RHALL_MSB (0x49)
  */
  uint8_t MAGrawADC[8];
  
  uint16_t DATA_RHALL;

  // Uncompensated DATA X/Y/Z
  int16_t DATA_X_UNCP;
  int16_t DATA_Y_UNCP;
  int16_t DATA_Z_UNCP;

  // Compensated DATA X/Y/Z
  int16_t DATA_X_COMP;
  int16_t DATA_Y_COMP;
  int16_t DATA_Z_COMP;



  // Leemos 8 bytes en modo burst a partir del registro BMM055_REGISTER_MAG_DATA (0x42) y los metemos en MAGrawADC[8]
  i2c_read_reg_to_buf(BMM055_ADDRESS, BMM055_REGISTER_MAG_DATA, &MAGrawADC, 8);
  //i2c_read_reg_to_buf(BMM055_ADDRESS, BMM055_REGISTER_MAG_DATA, &rawADC, 6);


  // Resolution: 
  //  13 bits for X/Y axis
  //  15 bits for Z   axis
  //  14 bits for RHALL    
  // Convertimos el contenido recibido a información magnética
  DATA_X_UNCP = (((uint16_t)MAGrawADC[1]) << 5) | (MAGrawADC[0] >> 3);   // X_MSB<<5 | X_LSB>>3  // Estamos concatenando de forma adecuada el contenido de los registros
  DATA_Y_UNCP = (((uint16_t)MAGrawADC[3]) << 5) | (MAGrawADC[2] >> 3);   // Y_MSB<<5 | Y_LSB>>3  // Estamos concatenando de forma adecuada el contenido de los registros
  DATA_Z_UNCP = (((uint16_t)MAGrawADC[5]) << 7) | (MAGrawADC[4] >> 1);   // Z_MSB<<7 | Z_LSB>>1  // Estamos concatenando de forma adecuada el contenido de los registros
  DATA_RHALL  = (((uint16_t)MAGrawADC[7]) << 6) | (MAGrawADC[6] >> 2);   // R_MSB<<6 | R_LSB>>2  // Estamos concatenando de forma adecuada el contenido de los registros

  // Compensamos los valores con los datos de la resistencia RHALL
  DATA_X_COMP = bmm055_compensate_X(DATA_X_UNCP, DATA_RHALL);
  DATA_Y_COMP = bmm055_compensate_Y(DATA_Y_UNCP, DATA_RHALL);
  DATA_Z_COMP = bmm055_compensate_Z(DATA_Z_UNCP, DATA_RHALL);

  // Le pasamos a MultiWii los valores del magnetómetro
  // Recordatorio: #define MAG_ORIENTATION(X, Y, Z)  {imu.magADC[ROLL]  = X; imu.magADC[PITCH]  = Y; imu.magADC[YAW]  = Z;}
  // Datos sin compensar
  // MAG_ORIENTATION(DATA_X_UNCP, DATA_Y_UNCP, DATA_Z_UNCP);
  // Datos compensados
  MAG_ORIENTATION(DATA_X_COMP, DATA_Y_COMP, DATA_Z_COMP);    

//   for (int i=0; i<8; ++i)
//    rawADC[i] = 0;


  float Pi = 3.14159;
  
  // Calculate the angle of the vector y,x
  float heading = (atan2(DATA_Y_COMP,DATA_X_COMP) * 180) / Pi;
  
  // Normalize to 0-360
  if (heading < 0)
  {
    heading = 360 + heading;
  }

  Serial.print("Compass Heading: ");
  Serial.println(heading);

#ifdef SERIAL_DEBUG_MAG
  Serial.print("X_UNCP: 0x");
  Serial.println(DATA_X_UNCP, HEX);
  Serial.print("Y_UNCP: 0x");
  Serial.println(DATA_Y_UNCP, HEX);
  Serial.print("Z_UNCP: 0x");
  Serial.println(DATA_Z_UNCP, HEX);

  Serial.print("X_COMP: ");
  Serial.println(DATA_X_COMP);
  Serial.print("Y_COMP: ");
  Serial.println(DATA_Y_COMP);
  Serial.print("Z_COMP: ");
  Serial.println(DATA_Z_COMP);

  Serial.print("RHALL: ");
  Serial.println(DATA_RHALL);
  Serial.println();

  Serial.print("imu.magADC[YAW]: ");
  Serial.println(imu.magADC[YAW]);
  Serial.println();
#endif // SERIAL_DEBUG_MAG

}
#endif // BMM055
