
/* Get tilt angles on X and Y, and rotation angle on Z
 * Angles are given in degrees, Serials on SSD1306 OLED
 * 
 * License: MIT
 */
#include <Wire.h>

#include <MPU6050_light.h>
 
 
MPU6050 mpu(Wire);
unsigned long timer = 0;
 
void setup() {
  Serial.begin(115200);                           // Ensure serial monitor set to this value also    
                         
  Wire.begin();
  mpu.begin();
 Serial.println(F("Calculating gyro offset, do not move MPU6050"));
        
  mpu.calcGyroOffsets();                          // This does the calibration
}
 
void loop() {
  mpu.update();  
  if((millis()-timer)>100)                         // print data every 10ms
  {                                           
                         
    Serial.print("P : ");
    Serial.println(mpu.getAngleX());
    Serial.print("R : ");
    Serial.println(mpu.getAngleY());
    Serial.print("Y : ");
    Serial.print(mpu.getAngleZ());
    Serial.println();                          // Serial data
    timer = millis();  
  }
}