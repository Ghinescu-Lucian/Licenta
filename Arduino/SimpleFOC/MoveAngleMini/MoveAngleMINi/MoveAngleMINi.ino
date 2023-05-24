
#include <SimpleFOC.h>
#include "Wire.h"
#include <MPU6050_light.h>

BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(9,10,11,8);

MPU6050 mpu(Wire);

//  Encoder(int encA, int encB , int cpr, int index)
Encoder encoder = Encoder(2, 3, 2048);
// interrupt routine intialisation
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}


void setup() {
  // put your setup code here, to run once:

// initialise encoder hardware
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


  // initialize motor
 motor.init();
  // monitoring port
  Serial.begin(115200);
//  mpu.begin();

  // set FOC loop to be used
  motor.controller = MotionControlType::angle;
  
  //align encoder and start FOC
  motor.initFOC();
  _delay(1000);

  Serial.println(F("\n Motor ready."));
  

}

float target_angle =0.0;
double accX;

void loop() {
  motor.loopFOC();

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  motor.move(target_angle);
  
  // mpu.update();
  //  accX= mpu.getAccX();
  //  Serial.print("AccX:"); Serial.print(accX); Serial.print(",");


  // communicate with the user
  serialReceiveUserCommand();
  // delay(25);
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


