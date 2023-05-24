
/*
  Maxim 4 pasi stanga dreapta
  Setpoint  = [-2,2]

**/

#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);


double Input,Setpoint, Output, Kp, Ki, Kd;
double lastError, integral = 0;

double lastInput = 0;
long time=0;


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
 

  
  int c=500;

  while(c > 0){
    c--;
    mpu.update();
    Input = mpu.getAngleY();
    //delay(5);
  }

  mpu.update();
  Input = mpu.getAngleY();
  Setpoint = Input;
  Serial.println(Setpoint);
  Kp = 5; // Proportional gain
//  Ki = 0.1; // Integral gain
//  Kd = 0.05; // Derivative gain



   Serial.println("Done!\n");

}

void loop() {
 
  double error = diff(Input); // Calculate the error
  integral += error; // Update the integral
  double derivative = error - lastError; // Calculate the derivative
  Output = Kp * error + Ki * integral + Kd * derivative; // Calculate the output
  lastError = error; // Update the last error

  if(Output > 500) Output = 500;
  if(Output < -500) Output = -500;

  Serial.print("Output:"); Serial.print(Output);Serial.println("");
  Serial.print("Input:"); Serial.print(Input);Serial.println("");
  Serial.print("Error:"); Serial.print(error);Serial.println("");





  int c=5;

  while(c > 0){
    c--;
    mpu.update();
    //Input = mpu.getAngleX();
    //delay(5);
  }

  mpu.update();
  Input = mpu.getAngleY(); // Read the encoder position
//  delay(10);

  


}

double diff(double x){
 double dif=0;

 dif = x - Setpoint;
  
 if( dif < 2 && dif > -2)

   return 0;

  return dif;
  
}
