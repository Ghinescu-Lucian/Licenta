
/*
MUTARE DE PE GYRO PE ACC
AHRS ATITUDE AND REF SYSTEM


**/
#include <SimpleFOC.h>
#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(10,11, 12, 7);

// encoder instance
Encoder encoder = Encoder(2, 3, 2048); // 2048
// BLDC motor & driver instance
BLDCMotor motor2 = BLDCMotor(7);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(9,5,6,8);;

// encoder instance
// Encoder encoder = Encoder(18, 19, 2048);
Encoder encoder2 = Encoder(18, 19, 2048);


double Input,Setpoint;
float target_angle=0.0;




// interrupt routine intialisation
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}
void doA2(){encoder2.handleA();}
void doB2(){encoder2.handleB();}

Commander command = Commander(Serial);
void doTarget(char* cmd) {
  float ramp=0;
  command.scalar(&ramp, cmd);
  motor.PID_velocity.output_ramp=ramp;
 
}

void setup() {


  Serial.begin(115200);
  // Serial.print(F("MPU6050 status: "));
  // Serial.println(status);
//  Initializare MPU6050

  Wire.begin();
  byte status = mpu.begin();
  while(status!=0){ } // stop everything if could not connect to MPU6050
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true,true); // gyroscope and accelero
  Serial.println("Done!\n");

  // Initializare motoare
 // Motor 1
  encoder.init();
  encoder.enableInterrupts(doA,doB);
  motor.linkSensor(&encoder);
  
  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);
  motor.voltage_sensor_align = 12;
  motor.voltage_limit = 12;

  motor.controller = MotionControlType::angle;
  motor.foc_modulation = FOCModulationType::SinePWM;
// velocity PID
  motor.PID_velocity.P = 0.5;
  motor.PID_velocity.I = 20;
  motor.PID_velocity.D = 0.01;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond11
  motor.PID_velocity.output_ramp = 2000;
   // velocity low pass filtering time constant
  motor.LPF_velocity.Tf = 0.01f;
  // velocity limit
  motor.velocity_limit = 25;

// angle PID
  // motor.P_angle.P = 43.2;
  // motor.P_angle.I = 80;
  // motor.P_angle.D = 0.003125f;
  // motor.P_angle.output_ramp=2500;
  // motor.LPF_angle = 0.001f;
   motor.P_angle.P = 20;
  // motor.P_angle.I = 80;
  // motor.P_angle.D = 0.003125f;
  motor.P_angle.output_ramp=2500;
  motor.LPF_angle = 0.001f;

// direction
  motor.zero_electric_angle = 4.03;
  motor.sensor_direction = CCW;

  motor.useMonitoring(Serial);
  motor.init();
  motor.initFOC();

// Motor 2
  encoder2.init();
  encoder2.enableInterrupts(doA2,doB2);
  motor2.linkSensor(&encoder2);
  
  driver2.voltage_power_supply = 12;
  driver2.init();
  motor2.linkDriver(&driver2);
  motor2.voltage_sensor_align = 6;
  motor2.voltage_limit = 6;

  motor2.controller = MotionControlType::angle;
  motor2.foc_modulation = FOCModulationType::SinePWM;
// velocity PID
  motor2.PID_velocity.P = 0.2f;
  motor2.PID_velocity.I = 10;
  motor2.PID_velocity.D = 0.0;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond11
  motor2.PID_velocity.output_ramp = 1000;
   // velocity low pass filtering time constant
  motor2.LPF_velocity.Tf = 0.01f;
// velocity limit
  motor2.velocity_limit=15;  

// angle PID
  motor2.P_angle.P = 15;
  motor2.P_angle.I = 0;//13.3333;
  motor2.P_angle.D = 0;//0.01875;
  motor2.P_angle.output_ramp=1000;
  motor2.LPF_angle.Tf=0.01f;

// direction
  motor2.zero_electric_angle = 3.74;
  motor2.sensor_direction = CW;

  motor2.useMonitoring(Serial);
  motor2.init();
  motor2.initFOC();



// aduc motoarele la pozitia 0.0
  int c=50;
  while(c>0){
    c--;
    motor.loopFOC();
    motor2.loopFOC();
    motor.move(0.0);
    motor2.move(0.0);
  }
  _delay(500);
   command.add('t', doTarget, "target ramp");
  // Comunicatia cu MPU6050
  c=50;
  while(c){
    c--;
    mpu.update();
    mpu.getAngleY();
    delay(10);
  }
  mpu.update();
  Input = mpu.getAngleY();
  Setpoint = Input;
  Serial.print("Setpoint: "); Serial.println(Setpoint);
  Serial.println("Done!");
  pinMode(23, OUTPUT);
  digitalWrite(23, HIGH);

}

float target_angle_roll =0.0;
float last_error=0.0;
unsigned long timer = 0;

void loop() {
 
 motor.loopFOC();
 motor2.loopFOC();
  
   mpu.update();
   if((millis()-timer > 25 )){
  Input = mpu.getAngleY();
  double error = diff(Input); 
   float move;
  // //  if( abs( error - last_error) < 0.001)
  if(last_error != 0)
    error =0;
  if(error != 0){
    move= -error/57.2968; // 56.7 57.4
  }

  else move =0.0;
  
  if(move < -0.45) move=-0.45;
  if (move > 0.45) move = 0.45;
  target_angle += move;
  if(target_angle > 0.60)  target_angle =0.60;
  if(target_angle < -0.40) target_angle =-0.40;
  //   Serial.print("Input:"); Serial.print(Input);Serial.println("");
     Serial.print("Tg:"); Serial.print(target_angle);Serial.println("");
     Serial.print("ENC:"); Serial.print(encoder.getAngle());Serial.println("");
  // Serial.print("Move:"); Serial.print(move);Serial.println("");
  last_error = error;
  timer = millis();
   }

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  motor.move(target_angle);
  motor2.move(0.0);
  // motor.move(0.0);

  command.run();
 
 

}

double diff(double x){
 double dif=0;

 dif = x - Setpoint;
  
 if( dif < 0.5 && dif > -0.5)

   return 0;

  return dif;
  
}
