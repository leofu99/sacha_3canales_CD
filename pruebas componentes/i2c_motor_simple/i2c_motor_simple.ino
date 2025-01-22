#include <Wire.h>   // Incluye librería para interfaz I2C
#include "PCF8574.h" // Librería para el PCF8574
int dir = 6;
int stepm = 5;
PCF8574 pcf8574(0x25); // Dirección del PCF8574



void setup() {

   for (int i = 0; i < 8; i++) {
    if(i!=3){
    pcf8574.pinMode(i, OUTPUT);
    }
  // put your setup code here, to run once:
pcf8574.digitalWrite(i, LOW);
}
pcf8574.digitalWrite(dir, HIGH);
}

void loop() {
  pcf8574.digitalWrite(stepm, LOW);
  delay(100);
  pcf8574.digitalWrite(stepm, HIGH);
  delay(100);
  
  

  // put your main code here, to run repeatedly:

}
