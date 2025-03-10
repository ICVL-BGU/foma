#define ANALOG_OUT_PIN 5 // Use a PWM-capable pin (D3, D5, D6, D9, etc.)

void setup() {
    Serial.begin(115200); // Start serial communication
    pinMode(ANALOG_OUT_PIN, OUTPUT);
    Serial.println("Arduino Nano 33 IoT Ready. Send a value (0-1) over Serial.");
}

void loop() {
    if (Serial.available()) {
        String received = Serial.readStringUntil('\n'); // Read data from Serial
        float value = received.toFloat(); // Convert to float

        if (value >= 0.0 && value <= 1.0) {
            int pwmValue = value * 255; // Scale 0-1 to 0-255 for PWM
            analogWrite(ANALOG_OUT_PIN, pwmValue);
            Serial.print("Set PWM to: ");
            Serial.println(pwmValue);
        } else {
            Serial.println("Invalid input. Please send a number between 0 and 1.");
        }
    }
}
