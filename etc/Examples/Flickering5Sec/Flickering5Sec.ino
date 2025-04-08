// Define the PWM output pin
const int pwmPin = 2; // Use pin 9 or another PWM-capable pin on Arduino Nano

void setup() {
  pinMode(pwmPin, OUTPUT); // Set the PWM pin as an output
}

void loop() {
  // Check if data is available to read
  digitalWrite(pwmPin, HIGH);
  delay(5000);
  digitalWrite(pwmPin, LOW);
  delay(5000);
}
