#include <SimpleFOC.h>

#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);



// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(10,11, 12, 7);

// encoder instance
Encoder encoder = Encoder(2, 3, 2048); // 2048

// BLDC motor & driver instance
BLDCMotor motor2 = BLDCMotor(7);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(9,5, 6, 8);

// encoder instance
Encoder encoder2 = Encoder(18, 19, 2048); // 2048

// Interrupt routine intialisation
// channel A and B callbacks
void doA() {
  encoder.handleA();
}
void doB() {
  encoder.handleB();
}
// channel A and B callbacks
void doA2() {
  encoder2.handleA();
}
void doB2() {
  encoder2.handleB();
}

// angle set point variable
float target_angle = 0;
double pitch_point=0, roll_point=0;

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) {
  command.scalar(&target_angle, cmd);
  //  mpu.update();
  //   Serial.print("Pitch:"); Serial.print((pitch_point - mpu.getAngleX())); Serial.print(",");
  //   Serial.print("Roll:"); Serial.print((roll_point - mpu.getAngleY())); Serial.println(",");

}

void doTargetP(char* cmd) {
  float p=0;
  command.scalar(&p, cmd);
  motor.P_angle.P =p;
}
void doTargetI(char* cmd) {
  float i =0;
  command.scalar(&i, cmd);
  motor.P_angle.I = i;
}
void doTargetD(char* cmd) {
  float d=0;
  command.scalar(&d, cmd);
  motor.P_angle.D=d;
}


void setup() {

  // use monitoring with serial
  Serial.begin(9600);
 

  // initialize encoder sensor hardware
  encoder.init();
  encoder.enableInterrupts(doA, doB);
  encoder2.init();
  encoder2.enableInterrupts(doA2, doB2);
  // link the motor to the sensor
  motor.linkSensor(&encoder);
  motor2.linkSensor(&encoder2);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();

  driver2.voltage_power_supply = 12;
  driver2.init();
  // link the motor and the driver
  motor.linkDriver(&driver);
  motor2.linkDriver(&driver2);

  // aligning voltage [V]
  motor.voltage_sensor_align = 6.5;

  // set motion control loop to be used
  motor.controller = MotionControlType::angle;
  motor2.controller = MotionControlType::angle;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  // contoller configuration
  // default parameters in defaults.h

  // velocity PI controller parameters
  motor.PID_velocity.P = 0.5;
  motor.PID_velocity.I = 20;
  motor.PID_velocity.D = 0.01;
  // default voltage_power_supply
  motor.voltage_limit = 10;
  // motor.voltage = 8;
  //motor.
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond11
  motor.PID_velocity.output_ramp = 1000;

  // velocity low pass filtering time constant
  motor.LPF_velocity.Tf = 0.01f;

  // angle P controller
  // motor.P_angle.P = 35;
  // motor.P_angle.I = 20;

  motor.P_angle.P = 43.2;
  motor.P_angle.I = 80;
  motor.P_angle.D = 0.003125f;
  motor.P_angle.output_ramp=1000;
  //  maximal velocity of the position control
  motor.velocity_limit = 15;

  // motor.zero_electric_angle =6.21;
  // motor.zero_electric_angle =0.11;
  // 3.4 . 4,23
  motor.zero_electric_angle =4.03;
  motor.sensor_direction=CCW;

   motor2.PID_velocity.P = 0.2f;//0.2f;
  motor2.PID_velocity.I = 20;
  motor2.PID_velocity.D = 0.0;
  // default voltage_power_supply
  motor2.voltage_limit = 12;
  motor2.voltage_sensor_align=6;

  motor2.PID_velocity.output_ramp = 1000;

  // velocity low pass filtering time constant
  motor2.LPF_velocity.Tf = 0.01f;
  motor2.velocity_limit=15;
  motor2.P_angle.P = 12;
  motor2.P_angle.I = 13.3333;
  motor2.P_angle.D = 0.01875;
  motor2.P_angle.output_ramp=3000;
  motor2.LPF_angle.Tf=0.01f;

  motor2.useMonitoring(Serial);
    motor2.zero_electric_angle=3.74;
  motor2.sensor_direction=CW;
  motor2.init();
  motor2.initFOC();

  // comment out if not needed
  motor.useMonitoring(Serial);


  // initialize motor
  motor.init();
  // align encoder and start FOC
  motor.initFOC();

  int c=0;
  while(c<200){
    c++;
    motor.loopFOC();
    motor2.loopFOC();
    motor.move(0.0);
    motor2.move(0.0);
  }

  // add target command T
   command.add('t', doTarget, "target angle");
   command.add('p', doTargetP, "target angle P");
    command.add('i', doTargetI, "target angle I" );
     command.add('d', doTargetD, "target angle D");
  command.add('t', doTarget, "target angle");
  motor.useMonitoring(Serial);

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target angle using serial terminal:"));
  // _delay(1000);
   Wire.begin();
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  delay(1000);
  int c2=0;
  while(c2<100){
    c2++;
    mpu.update();
    pitch_point = mpu.getAngleX();
  roll_point = mpu.getAngleY();
    delay(5);
  }
  pitch_point = mpu.getAngleX();
  roll_point = mpu.getAngleY();
   Serial.println("Done!\n");
}

int cnt =0;
unsigned long timer =0;

void loop() {
  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz
  motor.loopFOC();
  motor2.loopFOC();
  // cnt++;
  // mpu.update();
  // if(millis()-timer >50){
  //   target_angle=-target_angle;
  //   timer=millis();
  // }
  if(millis()-timer > 25 ){
    // cnt = 0;
    
    // Serial.print("Pitch:"); Serial.print((pitch_point - mpu.getAngleX())); Serial.print(",");
    // Serial.print("Roll:"); Serial.print((roll_point - mpu.getAngleY())); Serial.println(",");
     Serial.print("ENC:"); Serial.print(encoder.getPreciseAngle(),6); Serial.println(",");
     timer = millis();
    
  }
 
  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  motor.move(target_angle);
  // motor2.move(0);



  // function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!!
  // motor.monitor();

  // user communication
  command.run();
}