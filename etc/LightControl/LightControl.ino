// Define the PWM output pin
const int pwmPin = 9; // Use pin 9 or another PWM-capable pin on Arduino Nano

void setup() {
  Serial.begin(9600); // Start the serial communication at 9600 baud
  pinMode(pwmPin, OUTPUT); // Set the PWM pin as an output
}

void loop() {
  // Check if data is available to read
  if (Serial.available() > 0) {
    int receivedValue = Serial.parseInt(); // Read the incoming integer value

    // Ensure the received value is in the range 0-255
    if (receivedValue >= 0 && receivedValue <= 255) {
      analogWrite(pwmPin, receivedValue); // Output the value as PWM signal
      Serial.print("Output set to: "); // Optional feedback
      Serial.println(receivedValue);
    } else {
      Serial.println("Error: Value must be between 0 and 255.");
    }
  }
}
