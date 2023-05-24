#include <SimpleFOC.h>
#include "Wire.h"
#include <MPU6050_light.h>

BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(9,10,11,8);

MPU6050 mpu(Wire);

//  Encoder(int encA, int encB , int cpr, int index)
// 2035 da exact 2 pi

Encoder encoder = Encoder(2, 3, 2048);
// interrupt routine intialisation
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}

double Input, Setpoint;

void setup() {
  // put your setup code here, to run once:

// initialise encoder hardware
  //encoder.quadrature = Quadrature::OFF;
  // encoder.pullup = Pullup::USE_INTERN;
  encoder.init();

  // hardware interrupt enable
  encoder.enableInterrupts(doA, doB);
  // link the motor to the sensor
  motor.linkSensor(&encoder);
  motor.LPF_angle.Tf = 0.05; // linia de aur0.0.0

  // power supply voltage
  // default 12V
  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);
  // motor.PID_velocity.D=0.0;
  //  motor.PID_velocity.P=0.50;
  //   motor.PID_velocity.I=0;

  // motor.foc_modulation = FOCModulationType::SinePWM;

Serial.begin(115200);

  // initialize motor
  motor.voltage_limit= 6;
  motor.useMonitoring(Serial);
 motor.init();
 Serial.println("Motor init");
  // monitoring port
 

  // set FOC loop to be used
  //motor.controller = MotionControlType::angle_openloop;
  // motor.ser
  motor.controller = MotionControlType::angle;
  
  //align encoder and start FOC
  motor.initFOC();
  //motor.controller = MotionControlType::angle;
  _delay(1000);
  

  motor.loopFOC();
 // motor.move(0.02);
  delay(10);

  Serial.println(F("\n Motor ready."));
  

}

float target_angle =0.00;
double accX;

void loop() {
  motor.loopFOC();
//  Serial.println("Motor loopFOC");
 

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  motor.move(target_angle);
  


  // communicate with the user
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

double diff(double x){
 double dif=0;

 dif = x - Setpoint;
  
 if( dif < 2 && dif > -2)

   return 0;

  return dif;
  
}

