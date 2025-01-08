#include "DFRobot_GP8403.h"
DFRobot_GP8403 dac(&Wire,0x5F);

void setup() {
  Serial.begin(115200);
  while(dac.begin()!=0){
    Serial.println("init error");
    delay(1000);
   }
  Serial.println("init succeed");
  dac.setDACOutRange(dac.eOutputRange10V);//Set the output range as 0-10V
  // dac.setDACOutVoltage(300,0);//The DAC value for 3.5V output in OUT0 channel
  // delay(1000);
  // dac.store(); //Save the set 3.5V voltage inside the chip
}

void loop(){
  int i=100;
  while(true){
    dac.setDACOutVoltage(i,0);//The DAC value for 3.5V output in OUT0 channel
    delay(1000);
    dac.store(); //Save the set 3.5V voltage inside the chip
    Serial.println(i);
    i = (i+5)%3000;
  }
    
}