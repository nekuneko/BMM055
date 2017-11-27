#include <Arduino.h>
#include "BMM055.h"
#include "i2c_multiwii.h"


void setup() 
{
  Serial.begin(9600);
  Serial.flush(); // Limpia basura del buffer
  delay(1000);
  
  Serial.println("BMM055 NEKU test");
  
  i2c_init();
  Mag_init();
  
}

void loop() 
{
  // put your main code here, to run repeatedly:
 Device_Mag_getADC();
  delay(1000);
}
