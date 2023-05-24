#include <SimpleFOC.h>
#include "Wire.h"
#include <MPU6050_light.h>

// MPU6050 mpu(Wire);
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(9,10,11,8);

float target_angle =0.0;

Encoder encoder = Encoder(2, 3, 2048);
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
  // motor.velocity_limit = 7;
  motor.linkDriver(&driver);
  motor.voltage_limit = 5.5;
  motor.useMonitoring(Serial);
  // motor.foc_modulation = FOCModulationType::SinePWM;


  // initialize motor
 motor.init();
  // monitoring port

   // set FOC loop to be used
  motor.controller = MotionControlType::angle_openloop;

  // smooth lpf from sensor, Tf = time constant, 
  // motor.LPF_angle.Tf = 0.01;

  // PID values to tune in angle mode
  // motor.P_angle.P I D
  // encoder.getAngle
  // motor.P_angle.P=8;
  //  motor.P_angle.I=0.5;
  //   motor.P_angle.D=0.01;// tremura rau!!!!!!!!!!
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

    motor.zero_electric_angle = 0.0;
  
   Serial.println("Done!\n");

}



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
 
 // Serial.print("Angle "); Serial.print(encoder.getPreciseAngle());Serial.println("");


   serialReceiveUserCommand();


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


