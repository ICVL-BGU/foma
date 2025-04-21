#include <WiFiNINA.h>

const char* ssid = "YourWiFiSSID";  // Replace with your WiFi SSID
const char* password = "YourWiFiPassword"; // Replace with your WiFi Password

WiFiServer server(80); // Start server on port 80

#define ANALOG_OUT_PIN 5 // Use a PWM-capable pin (D3, D5, D6, D9, etc.)

void setup() {
    Serial.begin(115200);
    pinMode(ANALOG_OUT_PIN, OUTPUT);

    // Connect to WiFi
    Serial.print("Connecting to WiFi...");
    while (WiFi.begin(ssid, password) != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }

    Serial.println("\nConnected to WiFi!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP()); // Print IP to use in the browser

    server.begin();
}

void loop() {
    WiFiClient client = server.available(); // Check for incoming clients
    if (client) {
        String request = client.readStringUntil('\r'); // Read HTTP request
        client.flush();

        // Parse the value from the request
        if (request.indexOf("/?value=") != -1) {
            int startIndex = request.indexOf("=") + 1;
            int endIndex = request.indexOf(" ", startIndex);
            String valueStr = request.substring(startIndex, endIndex);
            float value = valueStr.toFloat();

            if (value >= 0.0 && value <= 1.0) {
                int pwmValue = value * 255; // Scale 0-1 to 0-255 for PWM
                analogWrite(ANALOG_OUT_PIN, pwmValue);
                Serial.print("PWM Output Set To: ");
                Serial.println(pwmValue);

                // Send response to client
                client.print("HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\n\r\n");
                client.print("Set PWM to: " + String(pwmValue));
            } else {
                client.print("HTTP/1.1 400 Bad Request\r\n\r\nInvalid Value");
            }
        }
        client.stop();
    }
}
