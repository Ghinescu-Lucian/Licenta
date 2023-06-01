
/*
MUTARE DE PE GYRO PE ACC
AHRS ATITUDE AND REF SYSTEM


**/
#include <SimpleFOC.h>
#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(9,5,6,8);
BLDCMotor motor2 = BLDCMotor(7);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(10,11,12,7);

double Input,Setpoint;
float target_velocity=0;
double min=0, max=0;

Encoder encoder = Encoder(18, 19, 2048);
Encoder encoder2 = Encoder(2,3,2048);
// interrupt routine intialisation
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}
void doA2(){encoder2.handleA();}
void doB2(){encoder2.handleB();}


Commander command = Commander(Serial);
// motor 1 params
void doTarget(char* cmd) {
  command.scalar(&target_velocity, cmd);
  Serial.println("Velocity");
  min= max = target_velocity;
  // motor.=ap;
}

void doTarget_AP(char* cmd) {
  float ap=0;
  command.scalar(&ap, cmd);
  Serial.println("M AP");
  motor.P_angle.P=ap;
 
}
void doTarget_AI(char* cmd) {
  float ai=0;
  command.scalar(&ai, cmd);
   Serial.println("M AI");
  motor.P_angle.I=ai;
 
}
void doTarget_AD(char* cmd) {
  float ad=0;
  command.scalar(&ad, cmd);
   Serial.println("M AD");
  motor.P_angle.D=ad;
 
}
void doTarget_VP(char* cmd) {
  float vp=0;
  command.scalar(&vp, cmd);
  Serial.println("M VP");
  motor.PID_velocity.P=vp;
 
}
void doTarget_VI(char* cmd) {
  float vi=0;
  command.scalar(&vi, cmd);
   Serial.println("M VI");
  motor.PID_velocity.I=vi;
 
}
void doTarget_VD(char* cmd) {
  float vd=0;
  command.scalar(&vd, cmd);
   Serial.println("M VD");
  motor.PID_velocity.D=vd;
 
}

void doTarget_Vramp(char* cmd) {
  float ramp=0;
  command.scalar(&ramp, cmd);
  Serial.println("V ramp");
  motor.PID_velocity.output_ramp=ramp;
 
}
void doTarget_Aramp(char* cmd) {
  float aramp=0;
  command.scalar(&aramp, cmd);
  Serial.println("A ramp");
  motor.P_angle.output_ramp=aramp;
 
}


void setup() {


 Serial.begin(115200);

 
  
 encoder.init();
  // hardware interrupt enable
  encoder.enableInterrupts(doA, doB);
  // link the motor to the sensor
  motor.linkSensor(&encoder);

  // power supply voltage
  // default 12V
  driver.voltage_power_supply = 12;
  driver.init();
 
  motor.linkDriver(&driver);

  motor.foc_modulation = FOCModulationType::SinePWM;


  // initialize motor
 motor.init();
  // monitoring port
  motor.useMonitoring(Serial);
   // set FOC loop to be used
  // motor.controller = MotionControlType::torque;
  motor.controller = MotionControlType::velocity;
  //  motor.controller = MotionControlType::angle_openloop;

   motor.PID_velocity.P = 0.2f;//0.2f;
  motor.PID_velocity.I = 20;
  motor.PID_velocity.D = 0.0;
  // default voltage_power_supply
  motor.voltage_limit = 6;
  motor.voltage_sensor_align=6;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 150;

  // velocity low pass filtering time constant
  motor.LPF_velocity.Tf = 0.01f;

  // angle P controller
  // motor.P_angle.P =7.5;
  // motor.P_angle.D = 0.01f;
  // motor.P_angle.output_ramp=1000;
  // motor.LPF_angle.Tf= 0.01f;
  //  maximal velocity of the position control
  motor.velocity_limit = 20;

  // angle P controller
//  motor.P_angle.P = 12;
//   motor.P_angle.I = 13.3333;
//   motor.P_angle.D = 0.01875;
//   motor.P_angle.output_ramp=300;

  // motor.P_angle.P = 1;
//   motor.P_angle.I = 13.3333;
//   motor.P_angle.D = 0.01875;
//   motor.P_angle.output_ramp=300;
  motor.LPF_angle.Tf=0.01f;

  


  // motor.zero_electric_angle=3.74;
  // motor.sensor_direction=CW;
  motor.initFOC();


encoder2.init();
  encoder2.enableInterrupts(doA2, doB2);
  motor2.linkSensor(&encoder2);
  
  driver2.voltage_power_supply = 12;
  driver2.init();
  motor2.linkDriver(&driver2);

  motor2.voltage_sensor_align = 8;
  motor2.voltage_limit = 10;

  motor2.controller = MotionControlType::angle;
  //  motor2.controller = MotionControlType::angle_openloop;
  motor2.foc_modulation = FOCModulationType::SinePWM;

//  Velocity PID
  motor2.PID_velocity.P = 0.5;
  motor2.PID_velocity.I = 20;
  motor2.PID_velocity.D = 0.01;

// jerk control using voltage voltage ramp
// default value is 300 volts per sec  ~ 0.3V per millisecond11
  motor2.PID_velocity.output_ramp = 300;
// velocity low pass filtering time constant
  motor2.LPF_velocity.Tf = 0.01f;
// velocity limit
  motor2.velocity_limit = 15;

// angle PID
  motor2.P_angle.P = 43.2;
  motor2.P_angle.I = 80;
  motor2.P_angle.D = 0.003125f;
  motor2.P_angle.output_ramp=1000;
  motor2.LPF_angle = 0.001f;


// direction
  motor2.zero_electric_angle = 4.03;
  motor2.sensor_direction = CCW;

  motor2.useMonitoring(Serial);
  motor2.init();
  motor2.initFOC();



  // aduc motorul in pozitia 0
  int c1=500;
  while(c1){
      // motor.loopFOC();
      motor2.loopFOC();
      // motor.move(0.0);
      motor2.move(0.0);
      c1--;
  }
  _delay(500);

  // incep comunicatia cu MPU6050
// commands for motor 1
 command.add('p', doTarget_AP, "target ap");
 command.add('i', doTarget_AI, "target ai");
 command.add('d', doTarget_AD, "target ad");
 command.add('r', doTarget_Aramp, "target a ramp");
 command.add('P', doTarget_VP, "target vp");
 command.add('I', doTarget_VI, "target vi");
 command.add('D', doTarget_VD, "target vd");
 command.add('R', doTarget_Vramp, "targ v ramp");
 command.add('t', doTarget, "velocity");

  
 

}

float target_angle =0.0;
float last_error=0.0;
unsigned long timer = 0;
double last_angle=0;
float max_move=0.5;
int ok=0;


void loop() {
 
 motor.loopFOC();
//  motor2.loopFOC();
  motor.PID_velocity.P=target_velocity;
 motor.move(5.0);
//  motor2.move(0.0);
 double enc = encoder.getVelocity();
 if( enc < min){
   min = enc;
 }
 else if(enc > max){
   max = enc;
 }
 Serial.print(target_velocity); Serial.print(" "); Serial.println(encoder.getVelocity());
//  Serial.print("Min:"); Serial.print(min); Serial.print("  Max:"); Serial.println(max);
 
 command.run();

}

double diff(double x){
 double dif=0;

 dif = x - Setpoint;
  
 if( dif < 0.3 && dif > -0.3)

   return 0;

  return dif;
  
}
