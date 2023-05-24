#include <SimpleFOC.h>
#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(9,10,11,8);

float Input,Setpoint;

Encoder encoder = Encoder(2, 3, 2048);
// interrupt routine intialisation
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}


void setup() {


 Serial.begin(115200);
 Wire.begin();
 motor.useMonitoring(Serial);
 
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  

 
 encoder.init();
  // hardware interrupt enable
  encoder.enableInterrupts(doA, doB);
  // link the motor to the sensor
  motor.linkSensor(&encoder);

  // power supply voltage
  // default 12V
  driver.voltage_power_supply = 12;
  driver.init();
  // motor.velocity_limit = 7;
  motor.linkDriver(&driver);
  motor.voltage_limit = 6;
  // motor.foc_modulation = FOCModulationType::SinePWM;


  // initialize motor
 motor.init();
  // monitoring port

   // set FOC loop to be used
  motor.controller = MotionControlType::angle;

  // smooth lpf from sensor, Tf = time constant, 
  motor.LPF_angle.Tf = 0.1;

  // PID values to tune in angle mode
  // motor.P_angle.P I D
  // encoder.getAngle
  // motor.PID_velocity.P=0.6;
  // motor.PID_velocity.I=5;
  // motor.LPF_velocity.Tf=0.01;
   motor.P_angle.P=5;
   motor.P_angle.I=1;
    motor.P_angle.D=0.0;//
    
    // motor.LPF_angle.Tf= 0.05;
  
  //align encoder and start FOC

  motor.initFOC();
  int c1=500;
  while(c1){
      motor.loopFOC();
      motor.move(0.0);
      c1--;
  }
  _delay(1000);
   c1=50;
  // while(c1){
  //    mpu.update();
  //    delay(15);
  //     c1--;
  // }
  // Input = Setpoint = mpu.getAngleX();
  // Serial.println(Input);
  _delay(1000);
  
   Serial.println("Done!\n");

}

float target_angle =0.0;

void loop() {
 
 motor.loopFOC();
  
  //  mpu.update();
  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
 // motor.P_angle.D=target_angle;
  motor.move(target_angle);
  
//  motor.
 
  Serial.print("Angle:"); Serial.print(encoder.getPreciseAngle());Serial.println(",");
   int b= 2;
   while(b){
         mpu.update();
        b--;
        }

  //   Input = mpu.getAngleX();
  //   double error = diff(Input);
    // Serial.print("MPUA:"); Serial.print(mpu.getAccAngleX());Serial.println(",");
    // Serial.print("MPU:"); Serial.print(mpu.getAngleX());Serial.println(",");

   serialReceiveUserCommand();


}

double diff(double x){
 double dif=0;

 dif = x - Setpoint;
//  Serial.println(dif);
  
 if( dif < 2 && dif > -2)

   return 0;

  return dif;
  
}

int cnt =0;
void serialReceiveUserCommand() {

  // a string to hold incoming data
  static String received_chars;

  while (Serial.available()) {
    // get the new byte:
    cnt++;
    char inChar = (char)Serial.read();
    // add it to the string buffer:
    received_chars += inChar;
    // end of user input
    if (inChar == '\n') {

      // change the motor target
      // if(cnt%2==0) target_angle =-3.14;
      // else target_angle = 3.14;
     
      target_angle = received_chars.toFloat();
      Serial.print("Target angle: ");
      Serial.println(target_angle);

      // reset the command buffer
      received_chars = "";
    }
  }
}


