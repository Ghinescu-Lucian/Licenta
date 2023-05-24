
/*

  Setpoint  = [-10,0]

**/

#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);


double Input, Output, Kp, Ki, Kd;
double lastError, integral = 0;

const int motorPin = 9; // Pin to control the motor
const int encoderPin = 2; // Pin to read the encoder position

void setup() {


 Serial.begin(9600);
  Wire.begin();
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
 //mpu.calcOffsets(true,true); // gyro and accelero
  Serial.println("Done!\n");

  

  mpu.update();
  Input = mpu.getAccX();
  Kp = 5; // Proportional gain
  Ki = 2; // Integral gain
  Kd = 1; // Derivative gain
}

void loop() {
  double error = diff(Input); // Calculate the error
  integral += error; // Update the integral
  double derivative = error - lastError; // Calculate the derivative
  Output = Kp * error + Ki * integral + Kd * derivative; // Calculate the output
  lastError = error; // Update the last error

  Serial.print("Output:"); Serial.print(Output);Serial.println("");
  mpu.update();
  Input = mpu.getAccX(); // Read the encoder position
  delay(50);
}

double diff(double x){
 // double dif=0;
  if( x < -10 || x>0){
    if(x<0)
      return  x+10;
    else return x;
  }
  return 0;
  
}
