#include "i2c_helper.h" // i2c_init()
#include "bmm055.h"

bmm055 mag;  // si se llama al constructor aqui, pierde el bootloader.

void setup() 
{
  i2c_init();
  Serial.begin(115200);
  while (!Serial); // Esperar a que se abra el serial
  Serial.println("BMM055 NEKU test");

  bool fail = mag.init();
  if (fail)
    Serial.println("ERROR, invalid CHIP ID");
    
  //mag.printTrimRegisters();

  //mag.getRawData();
  mag.printTrimRegisters();
  //while(true);
}

void loop() 
{
  unsigned long startTime = millis();
  mag.updateMagData();
  unsigned long elapsedTime = millis() - startTime;
  
  Serial.print("Elapsed measeure time: "); Serial.print(elapsedTime); Serial.println(" milliseconds"); 
  Serial.print("rawDataX: "); Serial.println(mag.rawDataX);
  Serial.print("rawDataY: "); Serial.println(mag.rawDataY);
  Serial.print("rawDataZ: "); Serial.println(mag.rawDataZ);
  Serial.print("x: "); Serial.println(mag.x());
  Serial.print("y: "); Serial.println(mag.y());
  Serial.print("z: "); Serial.println(mag.z());
  Serial.print("r: "); Serial.println(mag.r());
  Serial.print("Compass Heading: "); Serial.println(mag.getHeading());
  Serial.println();

  delay(1000);
   //mag.printRawData();
}
