
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

double Input_roll,Setpoint_roll;
double Input_pitch, Setpoint_pitch;


Encoder encoder = Encoder(18, 19, 2048);
Encoder encoder2 = Encoder(2,3,2048);

// interrupt routine intialisation
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}
void doA2(){encoder2.handleA();}
void doB2(){encoder2.handleB();}

Commander command = Commander(Serial);
// motor 1 params
void doTarget_AP(char* cmd) {
  float ap=0;
  command.scalar(&ap, cmd);
  Serial.println("M Angle P");
  motor.P_angle.P=ap;
 
}
void doTarget_AI(char* cmd) {
  float ai=0;
  command.scalar(&ai, cmd);
   Serial.println("M Angle I");
  motor.P_angle.I=ai;
 
}
void doTarget_AD(char* cmd) {
  float ad=0;
  command.scalar(&ad, cmd);
   Serial.println("M Angle D");
  motor.P_angle.D=ad;
 
}
void doTarget_VP(char* cmd) {
  float vp=0;
  command.scalar(&vp, cmd);
  Serial.println("M Velocity P");
  motor.PID_velocity.P=vp;
 
}
void doTarget_VI(char* cmd) {
  float vi=0;
  command.scalar(&vi, cmd);
   Serial.println("M Velocity I");
  motor.PID_velocity.I=vi;
 
}
void doTarget_VD(char* cmd) {
  float vd=0;
  command.scalar(&vd, cmd);
   Serial.println("M Velocity D");
  motor.PID_velocity.D=vd;
 
}

void doTarget_Vramp(char* cmd) {
  float ramp=0;
  command.scalar(&ramp, cmd);
  Serial.println("Velocity ramp");
  motor.PID_velocity.output_ramp=ramp;
 
}
void doTarget_Aramp(char* cmd) {
  float aramp=0;
  command.scalar(&aramp, cmd);
  Serial.println("Angle ramp");
  motor.P_angle.output_ramp=aramp;
 
}
// motor2 params
void doTarget_AP2(char* cmd) {
  float ap2=0;
  command.scalar(&ap2, cmd);
  Serial.println("M Angle P");
  motor2.P_angle.P=ap2;
 
}
void doTarget_AI2(char* cmd) {
  float ai2=0;
  command.scalar(&ai2, cmd);
   Serial.println("M Angle I");
  motor2.P_angle.I=ai2;
 
}
void doTarget_AD2(char* cmd) {
  float ad2=0;
  command.scalar(&ad2, cmd);
   Serial.println("M Angle D");
  motor2.P_angle.D=ad2;
 
}
void doTarget_VP2(char* cmd) {
  float vp2=0;
  command.scalar(&vp2, cmd);
  Serial.println("M Velocity P");
  motor2.PID_velocity.P = vp2;
 
}
void doTarget_VI2(char* cmd) {
  float vi2=0;
  command.scalar(&vi2, cmd);
   Serial.println("M Velocity I");
  motor2.PID_velocity.I=vi2;
 
}
void doTarget_VD2(char* cmd) {
  float vd2=0;
  command.scalar(&vd2, cmd);
   Serial.println("M Velocity D");
  motor2.PID_velocity.D=vd2;
 
}

void doTarget_Vramp2(char* cmd) {
  float ramp2=0;
  command.scalar(&ramp2, cmd);
  Serial.println("Velocity ramp");
  motor2.PID_velocity.output_ramp=ramp2;
 
}
void doTarget_Aramp2(char* cmd) {
  float aramp2=0;
  command.scalar(&aramp2, cmd);
  Serial.println("Angle ramp");
  motor2.P_angle.output_ramp=aramp2;
 
}


void setup() {


 Serial.begin(115200);

//  Start communication with MPU6050 through I2C
 
  Wire.begin();


  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  mpu.setFilterGyroCoef(0.99);
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true,true); // gyro and accelero
  Serial.println("Done!\n");

// Initialize encoder, driver and motor ( roll motor )

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


  
   // set FOC loop to be used
  motor.controller = MotionControlType::angle;

   motor.PID_velocity.P = 0.2f;//0.2f;
  motor.PID_velocity.I = 20;
  motor.PID_velocity.D = 0.0;
  // default voltage_power_supply
  motor.voltage_limit = 6;
  motor.voltage_sensor_align=6;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 1000;

  // velocity low pass filtering time constant
  motor.LPF_velocity.Tf = 0.01f;

  motor.velocity_limit = 15;

  // angle P controller
  motor.P_angle.P = 20;
  // motor.P_angle.I = 13.3333;
  // motor.P_angle.D = 0.01875;
  motor.P_angle.output_ramp=750;
  motor.LPF_angle.Tf=0.01f;


// Initialize the motor parameters
// &  monitoring port
 
  // motor.zero_electric_angle=3.74;
  // motor.sensor_direction=CW;
  motor.useMonitoring(Serial);
// initialize motor
  motor.init();
  motor.initFOC();

// Initialize encoder2, driver2 and motor2 ( pitch motor )
  encoder2.init();
  encoder2.enableInterrupts(doA2, doB2);
  motor2.linkSensor(&encoder2);
  
  driver2.voltage_power_supply = 12;
  driver2.init();
  motor2.linkDriver(&driver2);

  motor2.voltage_sensor_align = 8;
  motor2.voltage_limit = 10;

  motor2.controller = MotionControlType::angle;
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
  int c=500;
  while(c > 0){
      motor.loopFOC();
      motor2.loopFOC();
      motor.move(0.0);
      motor2.move(0.0);
      c--;
  }
  _delay(500);

// commands for motor 1
 command.add('p', doTarget_AP, "target angle p");
 command.add('i', doTarget_AI, "target angle i");
 command.add('d', doTarget_AD, "target angle d");
 command.add('r', doTarget_Aramp, "target a ramp");
 command.add('P', doTarget_VP, "target v p");
 command.add('I', doTarget_VI, "target v i");
 command.add('D', doTarget_VD, "target v d");
 command.add('R', doTarget_Vramp, "target v ramp");

// commands for motor 2
 command.add('o', doTarget_AP2, "target2 angle p");
 command.add('n', doTarget_AI2, "target2 angle i");
 command.add('e', doTarget_AD2, "target2 angle d");
 command.add('a', doTarget_Aramp2, "target2 a ramp");
 command.add('O', doTarget_VP2, "target2 v p");
 command.add('N', doTarget_VI2, "target2 v i");
 command.add('E', doTarget_VD2, "target2 v d");
 command.add('A', doTarget_Vramp2, "target2 v ramp");

  // incep comunicatia cu MPU6050
  
  c=50;

  while(c > 0){
    c--;
    mpu.update();
    Input_roll = mpu.getAngleX();
    Input_pitch = mpu.getAngleY();
    delay(20);
  }
  // calculez valoarea initiala
  mpu.update();
  Input_roll = mpu.getAngleX();
  Input_pitch = mpu.getAngleY();
  Setpoint_roll = Input_roll;
  Setpoint_pitch = Input_pitch;
  Serial.print("Pitch initial:");Serial.println(Setpoint_pitch);
  Serial.print("Roll initail:");Serial.println(Setpoint_roll);
  Serial.println("Done!\n");

}

float target_angle_roll = 0.0;
float target_angle_pitch = 0.0;
float last_error_roll= 0.0;
float last_error_pitch= 0.0;
unsigned long timer = 0;
double last_angle_roll = 0;
double last_angle_pitch = 0;
float max_move_roll = 0.7;
float max_move_pitch_hg = 0.5;
float max_move_pitch_lw = -0.35;

void loop() {
 
 motor.loopFOC();
 motor2.loopFOC();
  
   mpu.update();
  //  mpu.update();
   if((millis()-timer > 25 ) ){
     // roll error
     if( abs( encoder.getVelocity() ) < 2.5) {
        Input_roll = mpu.getAngleX();
        double error_roll = diff_roll(Input_roll); 
        float move_roll;
        //  if( abs( error - last_error) < 0.001)
        if(last_error_roll != 0)
          error_roll =0;
        if(error_roll != 0){
          move_roll = -error_roll/57.296; // 56
          // move = -error/56;
        }
        else move_roll =0.0;
        
        if(move_roll < -max_move_roll )  move_roll = -max_move_roll;
        if( move_roll > max_move_roll) move_roll = max_move_roll;
        target_angle_roll += move_roll;
        if(target_angle_roll > max_move_roll ) target_angle_roll = max_move_roll;
        if( target_angle_roll < -max_move_roll)  target_angle_roll = -max_move_roll;
        // float dif = abs(target_angle_roll - last_angle_roll);
        // if(dif > 0.2){
        //   if(last_angle_roll < target_angle_roll)
        //     target_angle_roll = last_angle_roll - 0.1;
        //   else 
        //     target_angle_roll = last_angle_roll + 0.1;
          // float coef = dif / 0.15 ;
          // float mv = coef * 0.07;
          // if(last_angle_roll  > target_angle_roll)
          //   target_angle_roll = target_angle_roll+mv;
          // else
          //   target_angle_roll = target_angle_roll-mv;
          // target_angle_roll = last_angle_roll;
        //   Serial.print("Depasire!");
        // }
        // mpu.gets
        // Serial.print("Input:"); Serial.print(Input);Serial.println("");   
         Serial.print("AngleX:"); Serial.print(mpu.getAngleX()); Serial.print(",");
  Serial.print("AngleY:"); Serial.print(mpu.getAngleY()); Serial.print(",");  
        // Serial.print("Tg_roll:"); Serial.print(target_angle_roll);Serial.println(""); //Serial.print("TgLast:"); Serial.print(last_angle);Serial.println("");
        // Serial.print("Move:"); Serial.print(move);Serial.println("");


        last_error_roll = error_roll;
        last_angle_roll=target_angle_roll;
     }
     if( abs(encoder2.getVelocity() )  < 2) {
        // error pitch
        Input_pitch = mpu.getAngleY();
        double error_pitch = diff_pitch(Input_pitch); 
        float move_pitch;
        // //  if( abs( error - last_error) < 0.001)
        if(last_error_pitch != 0)
          error_pitch =0;
        if(error_pitch != 0){
          move_pitch = -error_pitch/57.295; // 56.7 57.4
        }

        else move_pitch =0.0;
        
        if(move_pitch < max_move_pitch_lw) move_pitch = max_move_pitch_lw;
        if (move_pitch > max_move_pitch_hg) move_pitch = max_move_pitch_hg;
        target_angle_pitch += move_pitch;
        if(target_angle_pitch > max_move_pitch_hg)  target_angle_pitch = max_move_pitch_hg;
        if(target_angle_pitch < max_move_pitch_lw) target_angle_pitch = max_move_pitch_lw;
        //   Serial.print("Input:"); Serial.print(Input);Serial.println("");
          // Serial.print("Tg:"); Serial.print(target_angle);Serial.println("");
        // Serial.print("Move:"); Serial.print(move);Serial.println("");
        last_error_pitch = error_pitch;
     }
            
        timer = millis();
  }

//  motor.move(0.0);
 motor.move(target_angle_roll);

 motor2.move(target_angle_pitch);
  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  
 command.run();
  

}

double diff_roll(double x){
 double dif=0;

 dif = x - Setpoint_roll;
  
 if( dif < 0.8 && dif > -0.8)

   return 0;

  return dif;
  
}

double diff_pitch(double x){
 double dif=0;

 dif = x - Setpoint_pitch;
  
 if( dif < 0.5 && dif > -0.5)

   return 0;

  return dif;
  
}

