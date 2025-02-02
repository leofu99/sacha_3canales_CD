#include <Wire.h>   // Incluye librería para interfaz I2C
#include "PCF8574.h" // Librería para el PCF8574
int dir = 2;
int stepm = 4;
PCF8574 pcf8574(0x25); // Dirección del PCF8574



void setup() {
  pinMode(4, OUTPUT);
   pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);
   pinMode(7, OUTPUT);
    pinMode(8, OUTPUT);
   pinMode(9, OUTPUT);

   for (int i = 0; i < 8; i++) {
    
    pcf8574.pinMode(i, OUTPUT);
     pcf8574.digitalWrite(i, HIGH);
    
   

}
pcf8574.digitalWrite(dir, HIGH);
}

void loop() {
  pcf8574.digitalWrite(stepm, LOW);
  delay(10);
  pcf8574.digitalWrite(stepm, HIGH);
  delay(10);
  
  

  // put your main code here, to run repeatedly:

}
