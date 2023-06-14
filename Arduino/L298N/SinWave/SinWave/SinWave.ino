const int pin = 9; // choose the pin for the PWM output
const int frequency = 100; // choose the frequency of the PWM signal (in Hz)
int amplitude = 10; // set the amplitude of the sine wave (0-255)

void setup() {
  // set the pin as an output
  pinMode(pin, OUTPUT);
  // initialize the serial port
  Serial.begin(9600);
}

void loop() {
  for (int i = 0; i < 360; i++) { // loop through 0-360 degrees
    float angle = i * 3.14159 / 180.0; // convert to radians
    int value = amplitude * sin(angle) + 10; // calculate the PWM duty cycle
    analogWrite(pin, value); // output the PWM signal
    Serial.println(value); // send the value to the serial port
    //delayMicroseconds(10000 / (360 * frequency)); // delay for one cycle
  }
}
