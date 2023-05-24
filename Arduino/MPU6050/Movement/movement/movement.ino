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

double accX,accY,accZ;
double gyroX,gyroY,gyroZ;

long timer = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
 mpu.calcOffsets(true,true); // gyro and accelero
  Serial.println("Done!\n");
  
}

void loop() {
  mpu.update();


  if(millis() - timer > 100){ // print data every second
  
  accX= mpu.getAccX();
  accY= mpu.getAccY();
  accZ= mpu.getAccZ();
  gyroX = mpu.getGyroX();
  gyroY = mpu.getGyroY();
  gyroZ = mpu.getGyroZ();

double dif = mpu.getGyroXoffset()-gyroX;
//Serial.print("Offset:"); Serial.println(mpu.getGyroXoffset());
  if(dif > 45 || dif< -45){
    if(dif<0) Serial.println("Move to right!");
    else Serial.println("Move to left!");
  }
   
    timer = millis();
  }

}
