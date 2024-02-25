#include <Arduino.h>
#include <Wire.h>

void setup(){
  Wire.begin();
  Serial.begin(9600);

  while(Serial.read() != 13)
    ;
  
  Serial.println("Program has Started!!");
}

bool slavePresent(byte addr){
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

void loop(){
  Serial.println("I2C slave device list");

  for (byte adr = 1; adr < 127; adr++){
    if(slavePresent(adr)){
      if(adr < 16)  Serial.print("O");
      Serial.print(adr, HEX);
      Serial.print(" ");
    }
  }

  Serial.println("\nDone");
  delay(500);
}