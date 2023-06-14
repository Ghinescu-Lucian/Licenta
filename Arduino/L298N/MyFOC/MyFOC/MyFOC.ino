#include <L298N.h>
#include <FOC.h>


// Motor configurations
#define POLES_PAIRS 7 // number of poles pairs
#define MAX_VOLTAGE 24 // max voltage for the driver
#define MAX_CURRENT 4 // max current for the driver

// Define the L298N driver pins
#define IN1 2
#define IN2 3
#define EN 4

// Motor object
BLDCMotor motor = BLDCMotor(POLES_PAIRS);

// L298N driver object
L298N driver = L298N(IN1, IN2, EN, MAX_VOLTAGE, MAX_CURRENT);

// Sensorless FOC object
BLDCDriver3PWM driverFOC = BLDCDriver3PWM();

void setup() {
  // Set the sensor alignment and direction to sensorless
  motor.shaft_direction = Direction::CCW;
  motor.sensor_direction = Direction::CCW;
  motor.use_sensorless = true;
  
  // Initialize the sensorless FOC driver
  driverFOC.init(&motor, MAX_VOLTAGE);
  
  // Set the voltage and current limits
  driverFOC.voltage_limit = MAX_VOLTAGE;
  driverFOC.current_limit = MAX_CURRENT;
}

void loop() {
  // Control the motor using sensorless FOC
  driverFOC.loopFOC();
  
  // Set the L298N driver PWM value
  driver.setOutput(motor.pwm);
}
