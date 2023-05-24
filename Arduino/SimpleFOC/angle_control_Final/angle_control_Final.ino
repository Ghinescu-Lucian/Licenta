#include <SimpleFOC.h>



// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(9,5,6, 8);
// Stepper motor & driver instance
//StepperMotor motor = StepperMotor(50);
//StepperDriver4PWM driver = StepperDriver4PWM(9, 5, 10, 6,  8);

// encoder instance
Encoder encoder = Encoder(18, 19, 2048);

// Interrupt routine intialisation
// channel A and B callbacks
void doA() {
  encoder.handleA();
}
void doB() {
  encoder.handleB();
}

// angle set point variable
float target_angle = 0;
// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) {
  command.scalar(&target_angle, cmd);
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
  Serial.begin(115200);
  // Wire.begin(4);


  // initialize encoder sensor hardware
  encoder.init();
  encoder.enableInterrupts(doA, doB);
  // link the motor to the sensor
  motor.linkSensor(&encoder);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);

  // aligning voltage [V]
  motor.voltage_sensor_align = 6.5;

  // set motion control loop to be used
  motor.controller = MotionControlType::angle;
    motor.foc_modulation = FOCModulationType::SinePWM;

  // contoller configuration
  // default parameters in defaults.h

  // velocity PI controller parameters
  motor.PID_velocity.P = 0.4;
  motor.PID_velocity.I = 20;
  motor.PID_velocity.D = 0;
  // default voltage_power_supply
  motor.voltage_limit = 12;
  motor.voltage_sensor_align = 6;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 3000;
    //  maximal velocity of the position control
  motor.velocity_limit = 50;

  // velocity low pass filtering time constant
  motor.LPF_velocity.Tf = 0.01f;

  // angle P controller
  motor.P_angle.P = 12;
  motor.P_angle.I = 13.3333;
  motor.P_angle.D = 0.01875;
  motor.P_angle.output_ramp=250;
  motor.LPF_angle.Tf=0.01f;
  // encoder.getVelocity();



  // comment out if not needed
  motor.useMonitoring(Serial);


  // initialize motor
  motor.init();

  // motor.sensor_direction = CW;
  // motor.sensor_direction=CW;
  // motor.zero_electric_angle=5.01; // 3.78 

  // align encoder and start FOC
  motor.initFOC();

  // add target command T
  command.add('t', doTarget, "target angle");
   command.add('p', doTargetP, "target angle P");
    command.add('i', doTargetI, "target angle I" );
     command.add('d', doTargetD, "target angle D");
  motor.useMonitoring(Serial);

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target angle using serial terminal:"));
  Serial.println(motor.sensor_offset);
  _delay(1000);
}


unsigned long timer=0;
void loop() {
  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz
  motor.loopFOC();
 // }
 if(millis()-timer > 50){
  Serial.print("EncAng:");Serial.print(encoder.getAngle()); Serial.println(",");
  timer = millis();
 }

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  motor.move(target_angle);



  // function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!!
  // motor.monitor();

  // user communication
  command.run();
}