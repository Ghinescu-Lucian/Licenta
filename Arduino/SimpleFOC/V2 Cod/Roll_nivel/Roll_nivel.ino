
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
float target_angle =0.0;


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

 
  Wire.begin();


  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  

/*
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

  // motor.foc_modulation = FOCModulationType::SinePWM;


  // initialize motor
 motor.init();
  // monitoring port
  motor.useMonitoring(Serial);
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

  // angle P controller
  motor.P_angle.P =7.5;
  motor.P_angle.D = 0.01f;
  motor.P_angle.output_ramp=1000;
  motor.LPF_angle.Tf= 0.01f;
  //  maximal velocity of the position control
  motor.velocity_limit = 6;



  motor.initFOC();
*/

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
  motor.controller = MotionControlType::angle;
  //  motor.controller = MotionControlType::angle_openloop;

   motor.PID_velocity.P = 0.2f;//0.2f;
  motor.PID_velocity.I = 10;
  motor.PID_velocity.D = 0.0;
  // default voltage_power_supply
  motor.voltage_limit = 6;
  motor.voltage_sensor_align=6;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 1000;

  // velocity low pass filtering time constant
  motor.LPF_velocity.Tf = 0.01f;

  // angle P controller
  // motor.P_angle.P =7.5;
  // motor.P_angle.D = 0.01f;
  // motor.P_angle.output_ramp=1000;
  // motor.LPF_angle.Tf= 0.01f;
  //  maximal velocity of the position control
  motor.velocity_limit = 25;

  // angle P controller
//  motor.P_angle.P = 12;
//   motor.P_angle.I = 13.3333;
//   motor.P_angle.D = 0.01875;
//   motor.P_angle.output_ramp=300;

  motor.P_angle.P = 20;
//   motor.P_angle.I = 13.3333;
//   motor.P_angle.D = 0.01875;
  motor.P_angle.output_ramp=750;
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
      motor.loopFOC();
      motor2.loopFOC();
      motor.move(0.0);
      motor2.move(0.0);
      c1--;
  }

  float x = mpu.getAngleX();
  float x2, d1 = abs(-2.7 - x), d2;
  float tg=0;
  int ok=0;
  int dir=1;
  while( x < -2.75 || x > -2.65)
  {
    Serial.print("X:");Serial.println(x);
    motor.loopFOC();
    mpu.update();
    if(ok == 0){
      motor.move(0.05);
      if(encoder.getAngle() < 0.035 && encoder.getAngle() > 0.02 ){
        ok=1;
        x2 = mpu.getAngleX();
        d2 = abs(-2.7 - x2) ;
        if( d1 > d2) dir = -1; 
      }

    }
    else{
        
        x = mpu.getAngleX();
        tg = tg + dir*0.002;
    }
    motor.move(tg);

  }
  target_angle = tg;
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


  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true,true); // gyro and accelero
  Serial.println("Done!\n");
  int c=50;

  while(c > 0){
    c--;
    mpu.update();
    Input = mpu.getAngleX();
    delay(20);
  }
  // calculez valoarea initiala
  mpu.update();
  Input = mpu.getAngleX();
  Setpoint = Input;
  Serial.println(Setpoint);
    motor.useMonitoring(Serial);
   Serial.println("Done!\n");
   pinMode(23,OUTPUT);
   digitalWrite(23, HIGH);

}


float last_error=0.0;
unsigned long timer = 0;
double last_angle=0;
float max_move=0.7;
int ok=0;

void loop() {
 
 motor.loopFOC();
 motor2.loopFOC();
  // ok++;
  // if(ok%3 ==0){
  //   ok=1;
    mpu.update();
  // }
   
  //  mpu.update();
   if((millis()-timer > 15 ) ) {
    if( abs(encoder.getVelocity()) < 2.5){
            Input = mpu.getAngleX();
            double enc_dif = abs(target_angle - encoder.getPreciseAngle());
            if(enc_dif < 0.075){
                  double error = diff(Input);
                  // motor.loopFOC(); 
                  // motor2.loopFOC();
                  float move;
                  //  if( abs( error - last_error) < 0.001)
                  if(last_error != 0)
                    error =0;
                  if(error != 0){
                    // move= -error/57.296; // 56
                    move = -error/57.2968;
                  }
                  else move =0.0;
                  
                  if(move < -max_move )  move=-max_move;
                  if( move > max_move) move =max_move;
                  target_angle += move;
                  if(target_angle > max_move ) target_angle =max_move;
                  if( target_angle < -max_move)  target_angle = -max_move;
                  double df = target_angle - last_angle;

                  // while( df > 0.05 || df < -0.05){
                    // if( df > 0.07 || df < -0.07)
                    // target_angle = target_angle * 0.5;
                    // target_angle = target_angle * 0.5;
                    // if( target_angle < 0 && last_angle < 0 && last_angle > target_angle )
                    //      target_angle = target_angle+0.05f;
                    // else if( target_angle < 0 && last_angle < 0 && last_angle < target_angle )
                    //       target_angle = target_angle+0.05f;
                    // else if( target_angle > 0 && last_angle > 0 && last_angle < target_angle )
                    //       target_angle = target_angle-0.05f;
                    // else if( target_angle > 0 && last_angle > 0 && last_angle > target_angle )
                    //       target_angle = target_angle+0.05f;
                    // else 
                    //       target_angle -= 0.05;

                    // df = target_angle - last_angle;
                  // }
                  // if(abs(target_angle - last_angle) > 0.05){
                  //   target_angle=target_angle/2;
                    
                  // }
                  // mpu.gets
                  // Serial.print("Input:"); Serial.print(Input,5);Serial.println("");     
                  // Serial.print("Tg:"); Serial.print(target_angle);Serial.println(""); Serial.print("TgLast:"); Serial.print(last_angle);Serial.println("");
                  // Serial.print("Move:"); Serial.print(move);Serial.println("");

                  last_error = error;
                  last_angle=target_angle;
            }
        }
        timer = millis();
  }

 motor.move(target_angle);

  motor2.move(0.0);
 
  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code

// command.run();

}

double diff(double x){
 double dif=0;

 dif = x - Setpoint;
  
 if( dif < 0.8 && dif > -0.8)

   return 0;

  return dif;
  
}
