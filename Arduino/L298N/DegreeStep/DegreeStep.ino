
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


int phase1 = 9;
int phase2 = 10;
int phase3 = 11;
int speed = 9;
int direction = 1;
int contor =0;
int contor2 =0;

const int buttonPin1 = 2;
int buttonState1 = 0;

void setup() {



 pinMode(phase1, OUTPUT);
  pinMode(phase2, OUTPUT);
  pinMode(phase3, OUTPUT);
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
    Input = mpu.getAngleX();
    //delay(5);
  }

  mpu.update();
  Input = mpu.getAngleX();
  Setpoint = Input;
  Serial.println(Setpoint);
  Kp = 2; // Proportional gain
  Ki = 0.0; // Integral gain
  Kd = 0.0; // Derivative gain


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


  int steps =0;/// abs(Output/10);
  if(steps!=0){
      if(error > 0){
        direction = -1;
      }else  if (error < 0) direction = 1;
  }
  Serial.print("Steps:"); Serial.print(steps);Serial.println("");
  Serial.print("Direction:"); Serial.print(direction);Serial.println("");

  // steps = 280;
  // while(steps && error != 0){
  //     rotateMotor();
      
  //     error= diff(Input);
  //     steps-=1;
  //   }
   buttonState1 = digitalRead(buttonPin1);
  
  if(digitalRead(buttonPin1) == HIGH){
   Serial.print(digitalRead(buttonPin1));
    while(digitalRead(buttonPin1)== HIGH){}
    //  direction = -direction;
     Serial.print("Am citit butonul!: ");
     Serial.println(direction);
     rotateMotor();  
  }



  int c=5;

  while(c > 0){
    c--;
    mpu.update();
    //Input = mpu.getAngleX();
    //delay(5);
  }

  mpu.update();
  Input = mpu.getAngleX(); // Read the encoder position
//  delay(10);

  


}

double diff(double x){
 double dif=0;

 dif = x - Setpoint;
  
 if( dif < 2 && dif > -2)

   return 0;

  return dif;
  
}

void rotateMotor(){

  if(direction == 1){
    contor=contor+1;
   if(contor % 2 == 0){
      digitalWrite(phase1, HIGH); delay(speed);
      digitalWrite(phase3, LOW); delay(speed);
      digitalWrite(phase2, HIGH); delay(speed);
    }
    else{
      digitalWrite(phase1, LOW); delay(speed);
      digitalWrite(phase3, HIGH); delay(speed);
      digitalWrite(phase2, LOW); delay(speed);
   }
     

  }
  else{

    contor2++;
   if(contor2%2==0){

      digitalWrite(phase1, LOW); delay(speed);
      digitalWrite(phase2, HIGH); delay(speed);
      digitalWrite(phase3, LOW); delay(speed);
   }
   else{
      digitalWrite(phase1, HIGH); delay(speed);
      digitalWrite(phase2, LOW); delay(speed);
      digitalWrite(phase3, HIGH); delay(speed);
   }
      
  
  }


}
