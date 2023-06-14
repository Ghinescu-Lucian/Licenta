#include <SimpleFOC.h>
#include <MPU6050_light.h>
#include <Wire.h>


MPU6050 mpu(Wire);
double max = 0, min = 0;

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 8);
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
long cnt = 0;

// angle set point variable
float target_angle = 0;
// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) {
  command.scalar(&target_angle, cmd);
  min = target_angle * 57.2958;
  max = target_angle * 57.2958;
  cnt = 0;
}
float p = 15, i = 0, d = 0, r = 1000;
float P = 0.2, I = 10, D = 0, R = 1000;
void doTargetP(char* cmd) {
  // float p=0;
  command.scalar(&p, cmd);
  // motor.P_angle.P =p;
}
void doTargetI(char* cmd) {
  // float i =0;
  command.scalar(&i, cmd);
  // motor.P_angle.I = i;
}
void doTargetD(char* cmd) {
  // float d=0;
  command.scalar(&d, cmd);
  // motor.P_angle.D=d;
}

void doTargetPV(char* cmd) {
  // float p=0;
  command.scalar(&P, cmd);
  // motor.P_angle.P =p;
}
void doTargetIV(char* cmd) {
  // float i =0;
  command.scalar(&I, cmd);
  // motor.P_angle.I = i;
}
void doTargetDV(char* cmd) {
  // float d=0;
  command.scalar(&D, cmd);
  // motor.P_angle.D=d;
}
void doTargetr(char* cmd) {
  // float d=0;
  command.scalar(&r, cmd);
  // motor.P_angle.D=d;
}
void doTargetRV(char* cmd) {
  // float d=0;
  command.scalar(&R, cmd);
  // motor.P_angle.D=d;
}

double Input, Setpoint;

void setup() {

  // use monitoring with serial
  Serial.begin(115200);
  // Wire.begin(4);

  Wire.begin();
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) {}  // stop everything if could not connect to MPU6050

  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true, true);  // gyro and accelero
  Serial.println("Done!\n");
  int c = 50;
  while (c) {
    c--;
    mpu.update();
    delay(10);
  }
  mpu.update();
  Input = mpu.getAngleX();
  Setpoint = Input;
  Serial.print("Setpoint: ");
  Serial.println(Setpoint);
  Serial.println("Done!");

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
  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 20;
  motor.PID_velocity.D = 0.005;
  // default voltage_power_supply
  motor.voltage_limit = 12;
  motor.voltage_sensor_align = 6;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 1000;
  //  maximal velocity of the position control
  motor.velocity_limit = 25;

  // velocity low pass filtering time constant
  motor.LPF_velocity.Tf = 0.01f;

  // angle P controller
  motor.P_angle.P = 20;
  // motor.P_angle.I = 13.3333;
  // motor.P_angle.D = 0.01875;
  motor.P_angle.output_ramp = 1000;
  motor.LPF_angle.Tf = 0.01f;
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
  command.add('i', doTargetI, "target angle I");
  command.add('d', doTargetD, "target angle D");
  command.add('r', doTargetr, "target angle ramp");
  command.add('P', doTargetPV, "target velocity P");
  command.add('I', doTargetIV, "target velocity I");
  command.add('D', doTargetDV, "target veloity D");
  command.add('R', doTargetRV, "target veloity ramp");

  motor.useMonitoring(Serial);
  pinMode(23, OUTPUT);
  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target angle using serial terminal:"));
  Serial.println(motor.sensor_offset);
  digitalWrite(23, HIGH);
  // _delay(1000);
}


unsigned long timer = 0;
void loop() {
  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz
  motor.loopFOC();
  motor.P_angle.P = p;
  motor.P_angle.I = i;
  motor.P_angle.D = d;
  motor.P_angle.output_ramp = r;
  motor.PID_velocity.P = P;
  motor.PID_velocity.I = I;
  motor.PID_velocity.D = D;
  motor.PID_velocity.output_ramp = R;
  mpu.update();
  // Serial.print("Velocity:");Serial.print(encoder.getVelocity()); Serial.println(",");
  if ((millis() - timer > 25)) {  //} && abs(encoder.getVelocity()) < 0.1 ) {
                                  //    Input=mpu.getAngleX();
                                  //    if(Input < min) min = Input;
                                  //    else if(Input > max) max = Input;
    float enc = encoder.getAngle();
    // if(abs(enc -  target_angle) > 0.01 )
    //   cnt++;
    // else{
    //  Serial.print("CNT:");Serial.print(cnt); Serial.println(",");
    // }
    Serial.print("EncAng:");
    Serial.print(enc, 4);
    Serial.println(",");
    //   Serial.print("AngleX:");Serial.println(mpu.getAngleX());
    //   Serial.print("Min:");Serial.println(min);
    //   Serial.print("Max:");Serial.println(max);
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