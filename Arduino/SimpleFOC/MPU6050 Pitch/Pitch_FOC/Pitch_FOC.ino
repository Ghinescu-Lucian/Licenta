
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

double Input,Setpoint;


Encoder encoder = Encoder(18, 19, 2048);
// interrupt routine intialisation
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}


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
  motor.voltage_limit = 12;
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
  // aduc motorul in pozitia 0
  int c1=100;
  while(c1){
      motor.loopFOC();
      motor.move(0.0);
      c1--;
  }
  _delay(1000);

  // incep comunicatia cu MPU6050
  Wire.begin();


  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
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

}

float target_angle =0.0;
float last_error=0.0;
unsigned long timer = 0;

void loop() {
 
 motor.loopFOC();
  
   mpu.update();
  //  mpu.update();
   if((millis()-timer > 20 )){
  Input = mpu.getAngleX();
  double error = diff(Input); 
   float move;
  //  if( abs( error - last_error) < 0.001)
  if(last_error != 0)
    error =0;
  if(error != 0){
    move= -error/56; // 56
  }
  else move =0.0;
  if(target_angle > 1 || target_angle < -1) target_angle =0;
  target_angle += move;
    Serial.print("Input:"); Serial.print(Input);Serial.println("");
     Serial.print("Tg:"); Serial.print(target_angle);Serial.println("");
  Serial.print("Move:"); Serial.print(move);Serial.println("");
  last_error = error;
  timer = millis();
   }

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  motor.move(target_angle);
 
  // 

  //  serialReceiveUserCommand();


}

double diff(double x){
 double dif=0;

 dif = x - Setpoint;
  
 if( dif < 4.2 && dif > -4.2)

   return 0;

  return dif;
  
}
