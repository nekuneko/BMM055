#include "i2c_helper.h"
#include "bmm055.h"

bmm055 mag;  //pierde el bootloader, hay que llamar al constructor:

void setup() 
{
  i2c_init();

  Serial.begin(115200);
  while (!Serial); // Esperar a que se abra el serial
  Serial.println("BMM055 NEKU test");

  mag.init();
  
  // Suspend Mode
  Serial.println("Suspend Mode");
  mag.doSuspendMode();
  //printMagPowerStatus();
  //Serial.println();
  //delay(1000);
  
  // Sleep Mode
  Serial.println("Sleep Mode");
  mag.doSleepMode();
  //printMagPowerStatus();
  //Serial.println();
  //delay(1000);

  //mag.printDefaultValues();

  
  mag.printTrimRegisters();


/*
  Serial.println("Power On Reset");
  mag.doSuspendMode();
  mag.doSoftReset();
*/

//  Serial.print("DRDY: "); Serial.println(mag.getDRDY());
//  Serial.println();

  // OpMode Normal
  //mag.printControlInt1();
 // mag.enableChannel(BMM055_CHANNEL_X, false);
//  mag.enableChannel(BMM055_CHANNEL_Y, false);
//  mag.enableChannel(BMM055_CHANNEL_Z, false);
//  mag.printControlInt2();
  Serial.println("Normal mode");
  mag.doNormalMode();
  //mag.printSelfTest();
  //mag.printRawData();
  //mag.setSelfTest(true);
  //mag.printSelfTest();

  

  /*
  // Poner en normal operation mode
  controlOP &= ~(1u << 2);
  controlOP &= ~(1u << 1); 
  write8(BMM055_ADDR, BMM055_CTRL_OP, controlOP); 
  
  printControlOP();
  //printMagInterruptStatus();
  
  */
/*
    Serial.print("uint8_t:  "); Serial.println(sizeof(uint8_t));
    Serial.print("int8_t:   "); Serial.println(sizeof(int8_t));
    Serial.print("uint16_t: "); Serial.println(sizeof(uint16_t));
    Serial.print("int16_t:  "); Serial.println(sizeof(int16_t));
    Serial.print("uint32_t: "); Serial.println(sizeof(uint32_t));
    Serial.print("int32_t:  "); Serial.println(sizeof(int32_t));
    Serial.print("uint64_t: "); Serial.println(sizeof(uint64_t));
    Serial.print("int64_t:  "); Serial.println(sizeof(int64_t));
    Serial.print("float:    "); Serial.println(sizeof(float));
    */
}

void loop() 
{
 
  if (mag.getDRDY())
  {
    mag.getRawData();
    Serial.print("compensatedX: "); Serial.println(mag.getCompensatedX());
  }
   //mag.printRawData();
    
}
