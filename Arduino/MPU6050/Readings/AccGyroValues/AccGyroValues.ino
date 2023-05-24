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
  Serial.begin(115200);
  Wire.begin();
  Serial.print("Cv");
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
 //mpu.calcOffsets(true,true); // gyro and accelero
  Serial.println("Done!\n");
  
}

void loop() {
  mpu.update();


  if(millis() - timer > 10){ // print data every second
  
  accX= mpu.getAccX();
  accY= mpu.getAccY();
  accZ= mpu.getAccZ();
  gyroX = mpu.getGyroX();
  gyroY = mpu.getGyroY();
  gyroZ = mpu.getGyroZ();

  Serial.print("AccX:"); Serial.print(accX); Serial.print(",");
  Serial.print("AccY:"); Serial.print(accY); Serial.print(",");
  Serial.print("AccZ:"); Serial.print(accZ); Serial.print(",");

  Serial.print("GyroX:"); Serial.print(gyroX); Serial.print(",");
  Serial.print("GyroY:"); Serial.print(gyroY); Serial.print(",");
  Serial.print("GyroZ:"); Serial.print(gyroZ); Serial.print(",");

//  Serial.print("GyroXOffset:"); Serial.print( mpu.getAccXoffset()); Serial.print(",");
 

  Serial.print("AngleX:"); Serial.print(mpu.getAngleX()); Serial.print(",");
  Serial.print("AngleY:"); Serial.print(mpu.getAngleY()); Serial.print(",");
  Serial.print("AngleZ:"); Serial.print(mpu.getAngleZ());Serial.print(",");

  

  Serial.println("");
   
    timer = millis();
  }

}
