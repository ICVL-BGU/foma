#include "DFRobot_GP8403.h"
DFRobot_GP8403 dac1(&Wire,0x5F), dac2(&Wire,0x5E);

void setup() {
  Serial.begin(115200);
  while(dac1.begin()!=0 || dac2.begin()!=0){
    Serial.println("init error");
    delay(1000);
   }
  Serial.println("init succeed");
  dac1.setDACOutRange(dac1.eOutputRange10V);//Set the output range as 0-10V
  dac2.setDACOutRange(dac2.eOutputRange10V);//Set the output range as 0-10V
  dac1.setDACOutVoltage(0,0);//The DAC value for 3.5V output in OUT0 channel
  dac1.setDACOutVoltage(0,1);//The DAC value for 3.5V output in OUT0 channel
  dac2.setDACOutVoltage(0,0);//The DAC value for 3.5V output in OUT0 channel
  dac2.setDACOutVoltage(0,1);//The DAC value for 3.5V output in OUT0 channel
  delay(1000);
  dac1.store(); //Save the set 3.5V voltage inside the chip
  dac2.store(); //Save the set 3.5V voltage inside the chip
}

void loop(){
  if (Serial.available() > 0) {
    int receivedValue = Serial.parseInt(); // Read the incoming integer value

    // Ensure the received value is in the range 0-255
    if (receivedValue >= 0 && receivedValue <= 255) {
      // analogWrite(pwmPin, receivedValue); // Output the value as PWM signal
      int value = map(receivedValue, 0, 255, 0, 10000);
      set_all(value);
      Serial.print("Output set to: "); // Optional feedback
      Serial.println(value);
    } else {
      Serial.println("Error: Value must be between 0 and 255.");
    }
  }
}

void set_all(int value){
  dac1.setDACOutVoltage(value,0);//The DAC value for 3.5V output in OUT0 channel
  dac1.setDACOutVoltage(value,1);//The DAC value for 3.5V output in OUT0 channel
  dac2.setDACOutVoltage(value,0);//The DAC value for 3.5V output in OUT0 channel
  dac2.setDACOutVoltage(value,1);//The DAC value for 3.5V output in OUT0 channel
}