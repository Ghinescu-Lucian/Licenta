#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);

double accX,accY,accZ;
double gyroX,gyroY,gyroZ;

long timer = 0;


int phase1 = 9;
int phase2 = 10;
int phase3 = 11;
int speed = 10;
int direction = 1;
int contor =0;
// 1 = forward , 2 = backward


const int buttonPin1 = 2;
int buttonState1 = 0;


void setup() {
  pinMode(buttonPin1, INPUT);
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
  Serial.println("Done!\n");  
}

void loop(){
//  delay(500);
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
    if(dif<0) {
      Serial.println("Move to right!");
      direction = 1;
    }
    else {
      Serial.println("Move to left!");
      direction=-1;
    }
  }
   
    timer = millis();
  }

   rotateMotor();  
  
  
}

void rotateMotor(){

  if(direction == 1){
    contor=contor+1;
      digitalWrite(phase1, HIGH); delay(speed);
      digitalWrite(phase3, LOW); delay(speed);
      digitalWrite(phase2, HIGH); delay(speed);
      digitalWrite(phase1, LOW); delay(speed);
      digitalWrite(phase3, HIGH); delay(speed);
      digitalWrite(phase2, LOW); delay(speed);
     

  }
  else{
      digitalWrite(phase2, HIGH); delay(speed);
      digitalWrite(phase3, LOW); delay(speed);
      digitalWrite(phase1, HIGH); delay(speed);
      digitalWrite(phase2, LOW); delay(speed);
      digitalWrite(phase3, HIGH); delay(speed);
      digitalWrite(phase1, LOW); delay(speed);
      }

}