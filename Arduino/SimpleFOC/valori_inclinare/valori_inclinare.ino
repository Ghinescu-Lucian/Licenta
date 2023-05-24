
/*
  Maxim 4 pasi stanga dreapta
  Setpoint  = [-2,2]

**/
#include <SimpleFOC.h>
#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(9,10,11,8);

double Input,Setpoint, Output, Kp, Ki, Kd;
double lastError, integral = 0;

Encoder encoder = Encoder(2, 3, 2048);
// interrupt routine intialisation
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}


void setup() {


 Serial.begin(115200);
  Wire.begin();


  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true,true); // gyro and accelero
  Serial.println("Done!\n");

 


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
  // motor.voltage_limit = 6;
  // motor.foc_modulation = FOCModulationType::SinePWM;


  // initialize motor
 motor.init();
  // monitoring port

   // set FOC loop to be used
  motor.controller = MotionControlType::angle;

   motor.PID_velocity.P = 0.2f;
  motor.PID_velocity.I = 20;
  motor.PID_velocity.D = 0;
  // default voltage_power_supply
  motor.voltage_limit = 6;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 1000;

  // velocity low pass filtering time constant
  motor.LPF_velocity.Tf = 0.01f;

  // angle P controller
  motor.P_angle.P =8;
  motor.P_angle.D = 0.02;
  motor.P_angle.output_ramp=1000;
  //  maximal velocity of the position control
  motor.velocity_limit = 6;



  motor.initFOC();
  int c1=100;
  while(c1){
      motor.loopFOC();
      motor.move(0.0);
      c1--;
  }
  _delay(1000);


  
  int c=50;

  while(c > 0){
    c--;
    mpu.update();
    Input = mpu.getAngleX();
    delay(5);
  }

  mpu.update();
  Input = mpu.getAngleX();
  Setpoint = Input;
  Serial.println(Setpoint);
  Kp = 5; // Proportional gain
//  Ki = 0.1; // Integral gain
//  Kd = 0.05; // Derivative gain

   Serial.println("Done!\n");

}

float target_angle =0.0;
unsigned long timer = 0;

void loop() {
 
 motor.loopFOC();
  
   mpu.update();
   if((millis()-timer >10)){
  Input = mpu.getAngleX();
  double error = diff(Input); 
  //  float move;
  // if(error != 0){
  //   move= -error/50; // 56
  // }
  // else move =0.0;
  // target_angle = move;
    Serial.print("Error:"); Serial.print(error);Serial.println("");
  
  // Serial.print("Move:"); Serial.print(move);Serial.println("");
   Serial.print("Enc:"); Serial.print(encoder.getAngle());Serial.println("");
  
  
  timer = millis();
   }

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  motor.move(target_angle);
 
  // 

   serialReceiveUserCommand();


}

double diff(double x){
 double dif=0;

 dif = x - Setpoint;
  
//  if( dif < 2 && dif > -2)

//    return 0;

  return dif;
  
}

void serialReceiveUserCommand() {

  // a string to hold incoming data
  static String received_chars;

  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the string buffer:
    received_chars += inChar;
    // end of user input
    if (inChar == '\n') {

      // change the motor target
      target_angle = received_chars.toFloat();
      Serial.print("Target angle: ");
      Serial.println(target_angle);

      // reset the command buffer
      received_chars = "";
    }
  }
}

