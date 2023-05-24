
/*
  Maxim 4 pasi stanga dreapta
  Setpoint  = [-2,2]

**/
#include <SimpleFOC.h>
#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);
BLDCMotor motor = BLDCMotor(7);
BLDCMotor motor1 = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(9,5,6,8);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(10,11,12,7);

double Input,Setpoint, Output, Kp, Ki, Kd;
double lastError, integral = 0;

Encoder encoder = Encoder(18, 19, 2048);
Encoder encoder1 = Encoder(2, 3, 2048);
// interrupt routine intialisation
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}
void doA1(){encoder1.handleA();}
void doB1(){encoder1.handleB();}

// angle set point variable
float target_angle_motor2 = 0;
// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) {
  command.scalar(&target_angle_motor2, cmd);
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
 encoder1.init();
  // hardware interrupt enable
  encoder.enableInterrupts(doA, doB);
  encoder1.enableInterrupts(doA1, doB1);
  // link the motor to the sensor
  motor.linkSensor(&encoder);
  motor1.linkSensor(&encoder1);

  // power supply voltage
  // default 12V
  driver.voltage_power_supply = 12;
  driver1.voltage_power_supply = 12;
  driver.init();
  driver1.init();
  // motor.velocity_limit = 7;
  motor.linkDriver(&driver);
  motor1.linkDriver(&driver1);
  // motor.voltage_limit = 6;
  // motor.foc_modulation = FOCModulationType::SinePWM;



  // aligning voltage [V]
  motor1.voltage_sensor_align = 3;

  // set motion control loop to be used
  motor1.controller = MotionControlType::angle;

  // contoller configuration
  // default parameters in defaults.h

  // velocity PI controller parameters
  motor1.PID_velocity.P = 0.2f;
  motor1.PID_velocity.I = 20;
  motor1.PID_velocity.D = 0;
  // default voltage_power_supply
  motor1.voltage_limit = 7;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor1.PID_velocity.output_ramp = 1100;

  // velocity low pass filtering time constant
  motor1.LPF_velocity.Tf = 0.01f;

  // angle P controller
  motor1.P_angle.P = 15;
  //  maximal velocity of the position control
  motor1.velocity_limit = 5;



  // comment out if not needed
  motor1.useMonitoring(Serial);

  // initialize motor
  motor1.init();
  // align encoder and start FOC
  // motor1.zero_electric_angle =6.14;
  // motor1.sensor_direction=CCW;
    motor1.zero_electric_angle =3.89;
  motor1.sensor_direction=CCW;
  motor1.initFOC();

  // add target command T
  command.add('T', doTarget, "target angle");












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
    motor.useMonitoring(Serial);
   Serial.println("Done!\n");

}

float target_angle =0.0;
float last_error=0.0;
unsigned long timer = 0;

void loop() {
 
 motor.loopFOC();
 motor1.loopFOC();
  
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
  if(move >1 ) move =1;
  else if ( move <-1) move =-1;
    Serial.print("Error:"); Serial.print(error);Serial.println("");
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
  motor1.move(target_angle_motor2);
 
  // 

  //  serialReceiveUserCommand();
  command.run();


}

double diff(double x){
 double dif=0;

 dif = x - Setpoint;
  
 if( dif < 4.2 && dif > -4.2)

   return 0;

  return dif;
  
}

// void serialReceiveUserCommand() {

//   // a string to hold incoming data
//   static String received_chars;

//   while (Serial.available()) {
//     // get the new byte:
//     char inChar = (char)Serial.read();
//     // add it to the string buffer:
//     received_chars += inChar;
//     // end of user input
//     if (inChar == '\n') {

//       // change the motor target
//       target_angle = received_chars.toFloat();
//       Serial.print("Target angle: ");
//       Serial.println(target_angle);

//       // reset the command buffer
//       received_chars = "";
//     }
//   }
// }

