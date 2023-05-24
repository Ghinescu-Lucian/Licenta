
/*
MUTARE DE PE GYRO PE ACC
AHRS ATITUDE AND REF SYSTEM


**/
#include <SimpleFOC.h>
#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);

// BLDC motor & driver instance

BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(9,5,6,8);

BLDCMotor motor2 = BLDCMotor(7);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(10,11, 12, 7);

// encoder instance
Encoder encoder = Encoder(18, 19, 2048);
Encoder encoder2 = Encoder(2, 3, 2048);


double Input_roll,Setpoint_roll;
double Input_pitch, Setpoint_pitch;
float target_angle_pitch=0.0;
float target_angle_roll =0.0;




// interrupt routine intialisation
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}
void doA2(){encoder2.handleA();}
void doB2(){encoder2.handleB();}

Commander command = Commander(Serial);
void doTarget(char* cmd) {
  command.scalar(&target_angle_pitch, cmd);
 
}

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
 encoder2.init();
  // hardware interrupt enable
  encoder.enableInterrupts(doA, doB);
  encoder2.enableInterrupts(doA2, doB2);
  // link the motor to the sensor
  motor.linkSensor(&encoder);
  motor2.linkSensor(&encoder2);

  // power supply voltage
  // default 12V
  driver.voltage_power_supply = 12;
  driver2.voltage_power_supply=12;
  driver.init();
  driver2.init();
 
  motor.linkDriver(&driver);
  motor2.linkDriver(&driver2);

  // motor.foc_modulation = FOCModulationType::SinePWM;


  // initialize motor 1
 motor.init();
  // monitoring port
  motor.useMonitoring(Serial);
   // set FOC loop to be used
  motor.controller = MotionControlType::angle;

   motor.PID_velocity.P = 0.2;//0.2f;
  motor.PID_velocity.I = 20;
  motor.PID_velocity.D = 0.0;
  // default voltage_power_supply
  motor.voltage_limit = 6;
  motor.voltage_sensor_align=6;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 3000;

  // velocity low pass filtering time constant
  motor.LPF_velocity.Tf = 0.01f;

  // angle P controller
  motor.P_angle.P =8;
  // motor.P_angle.I =7/.5;
  motor.P_angle.D = 0.01f;
  motor.P_angle.output_ramp=1000;
  motor.LPF_angle.Tf= 0.01f;
  //  maximal velocity of the position control
  motor.velocity_limit = 6;


  motor.zero_electric_angle=1.85;
  motor.sensor_direction=CW;
  motor.initFOC();
  // aduc motorul in pozitia 0
  int c1=500;
  while(c1){
      motor.loopFOC();
      motor.move(0.0);
      c1--;
  }

  // intialize motor2

  // aligning voltage [V]
  motor2.voltage_sensor_align = 6.5;

  // set motion control loop to be used
  motor2.controller = MotionControlType::angle;

  // contoller configuration
  // default parameters in defaults.h

  // velocity PID controller parameters
  motor2.PID_velocity.P = 1;
  motor2.PID_velocity.I =20;
  motor2.PID_velocity.D = 0.012;
  motor2.velocity_limit = 10;
  // default voltage_power_supply
  motor2.voltage_limit = 10;
  //motor.
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor2.PID_velocity.output_ramp = 3000;

  // velocity low pass filtering time constant
  motor2.LPF_velocity.Tf = 0.01f;

  // angle P controller
  motor2.P_angle.P = 15;
  motor2.P_angle.I = 20;
  //  maximal velocity of the position control
  

  // motor.zero_electric_angle =6.21;
  // motor.zero_electric_angle =0.11;
  // 3.4 . 4,23
  motor2.zero_electric_angle =4.23;
  motor2.sensor_direction=CCW;


  // comment out if not needed
  motor2.useMonitoring(Serial);

  // initialize motor
  motor2.init();
  // align encoder and start FOC
  motor2.initFOC();

  int c=0;
  while(c<200){
    c++;
    motor2.loopFOC();
    motor2.move(0.0);
  }


  _delay(1000);

  // incep comunicatia cu MPU6050



  
   c=50;

  while(c > 0){
    c--;
    mpu.update();
    Input_pitch = mpu.getAngleY();
    Input_roll = mpu.getAngleX();
    delay(20);
  }
  // calculez valoarea initiala
  mpu.update();
  Input_pitch = mpu.getAngleY();
  Setpoint_pitch = Input_pitch;
  Input_roll = mpu.getAngleX();
  Setpoint_roll = Input_roll;
  Serial.println(Setpoint_pitch);
  Serial.println(Setpoint_roll);
  motor.useMonitoring(Serial);
  Serial.println("Done!\n");
  command.add('t', doTarget, "target angle");

}

// float target_angle_roll =0.0;
float last_error_pitch=0.0;
float last_error_roll=0.0;
unsigned long timer = 0;

void loop() {
 
 motor.loopFOC();
 motor2.loopFOC();
  
   mpu.update();
  //  mpu.update();
   if((millis()-timer > 20 )){
  Input_roll = mpu.getAngleX();
  Input_pitch = mpu.getAngleY();

  double error_pitch = diff_pitch(Input_pitch); 
  double error_roll = diff_roll(Input_roll);
   float move_roll=0, move_pitch=0;
  // //  if( abs( error - last_error) < 0.001)
  if(last_error_pitch != 0)
    error_pitch =0;
  if(error_pitch != 0){
    move_pitch= -error_pitch/56; ///57.4; // 56.7
  }

  else move_pitch =0.0;
  if(move_pitch < -0.45) move_pitch=-0.45;
  if (move_pitch > 0.45) move_pitch = 0.45;
  
  

  if(last_error_roll != 0)   
    error_roll =0;
  if(error_roll != 0){
    move_roll= -error_roll/56; // 56
  }
  else move_roll =0.0;
        
  if(move_roll < -0.7 )  move_roll=-0.7;
  if( move_roll > 0.7) move_roll =0.7;

 
target_angle_roll += move_roll;
target_angle_pitch += move_pitch;

 

  // if(abs(move_roll) > abs(move_pitch))
  //     target_angle_roll += move_roll;
  // else target_angle_pitch += move_pitch;

  
  if(target_angle_pitch > 0.65)  target_angle_pitch =0.65;
  if(target_angle_pitch < -0.35) target_angle_pitch =-0.35;
  if(target_angle_roll > 0.7 ) target_angle_roll =0.7;
  if( target_angle_roll < -0.7)  target_angle_roll = -0.7;

    Serial.print("Input:"); Serial.print(Input_pitch);Serial.println("");
     Serial.print("Tg:"); Serial.print(target_angle_pitch);Serial.println("");
  Serial.print("Move:"); Serial.print(move_pitch);Serial.println("");
  last_error_pitch = error_pitch;
  last_error_roll = error_roll;
  timer = millis();

    motor.move(target_angle_roll);
  // motor2.move(0);
    motor2.move(target_angle_pitch);

   }

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  // motor.move(0);
//   motor.move(target_angle_roll);
//  // motor2.move(0);
//  motor2.move(target_angle_pitch);
 
  // 

  //  serialReceiveUserCommand();


}

double diff_roll(double x){
 double dif=0;

 dif = x - Setpoint_roll;
  
 if( dif < 3.2 && dif > -3.2)

   return 0;

  return dif;
  
}
double diff_pitch(double x){
 double dif=0;

 dif = x - Setpoint_pitch;
  
 if( dif < 2.2 && dif > -2.2)

   return 0;

  return dif;
  
}
