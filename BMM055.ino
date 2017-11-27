#include "i2c_adafruit.h"
#include "bmm055.h"

bmm055 mag;

void setup() 
{
  i2c_init();

  Serial.begin(115200);
  while (!Serial); // Esperar a que se abra el serial
  Serial.println("BMM055 NEKU test");

  
  // Suspend Mode
  Serial.println("Suspend Mode");
  mag.setSuspendMode();
  //printMagPowerStatus();
  //Serial.println();
  //delay(1000);
  
  // Sleep Mode
  Serial.println("Sleep Mode");
  mag.setSleepMode();
  //printMagPowerStatus();
  //Serial.println();
  //delay(1000);

  // OpMode Normal
  mag.printMagControlINT1();
 

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
