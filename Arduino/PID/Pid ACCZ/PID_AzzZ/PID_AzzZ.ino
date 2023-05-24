/* Get all possible data from MPU6050
 * Accelerometer values are given as multiple of the gravity [1g = 9.81 m/s²]
 * Gyro values are given in deg/s
 * Angles are given in degrees
 * Note that X and Y are tilt angles and not pitch/roll.
 *
 * License: MIT
 */

#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);



double accZ;
//double accZ_target;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  byte status = mpu.begin();
  Serial.print("MPU6050 status: ");
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
 // Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
// mpu.calcOffsets(true,true); // gyro and accelero
 int c= 10;
 while(c){
   mpu.update();
   c--;
   delay(10);
 }
// accZ_target = mpu.getAccZ();
  Serial.println("Done!\n");
  
}



void loop() {
  mpu.update();
  
  
  accZ= mpu.getAccZ();
  if(accZ < 0.98){
    // am coborat
    if(accZ <0.8)
      {
        // tremurat
        Serial.println("Tremurat C");
      }
    else{
      // coborat voluntar

    }
  }
  else if( accZ > 1.04){
    // am urcat
      if( accZ > 1.18){
      // tremurat
      Serial.println("Tremurat U");
      }
      else{
        // urcat voluntar
      }
    
  }
  
  Serial.print("AccZ:"); Serial.print(accZ); Serial.print(",");  
  Serial.print("A:"); Serial.print(1); Serial.print(",");


  Serial.println("");
  int c = 10;
  while(c){
    c--;
    mpu.update();
    delay(1);
  }
   

}
