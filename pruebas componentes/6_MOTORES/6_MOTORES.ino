#include <Wire.h>   // Incluye librería para interfaz I2C
#include "PCF8574.h" // Librería para el PCF8574
PCF8574 i2cHall(0x23); // Dirección del PCF8574 de los sensores Hall
int motorDelay = 1;
int sensor = 1;
int motores[6][4] = {
  {0, 4, 5,0},  // Motor 1: Directo, pines 4 y 5
  {0, 6, 7,1},  // Motor 2: Directo, pines 6 y 7
  {0, 8, 9,2},  // Motor 3: Directo, pines 8 y 9
  {1, 0, 1,4},  // Motor 4: I2C, pines 0 y 1 en PCF8574
  {1, 2, 4,5},  // Motor 5: I2C, pines 2 y 4 en PCF8574
  {1, 5, 6,6}   // Motor 6: I2C, pines 5 y 6 en PCF8574
};
PCF8574 pcf8574(0x25); // Dirección del PCF8574



void setup() {
  Serial.begin(9600);
  
  for (int i = 0; i < 7; i++) {
   
    if (motores[i][0] == 0) {  // Si es directo
      pinMode(motores[i][1], OUTPUT);
      pinMode(motores[i][2], OUTPUT);
      digitalWrite(motores[i][1], LOW);
      digitalWrite(motores[i][2], LOW);
      
    } else {
      pcf8574.pinMode(motores[i][1], OUTPUT);
      pcf8574.pinMode(motores[i][2], OUTPUT);
      pcf8574.digitalWrite(motores[i][1], LOW);
      pcf8574.digitalWrite(motores[i][2], LOW);
    }
     i2cHall.pinMode(motores[i][3], INPUT);
    
    }

   
  
}

void loop() {

  
  
    for (int i = 0; i < 6; i++) {
    if (motores[i][0] == 0) {  // Si es directo
      /*for (int j = 0; j < 200; j++) {
        digitalWrite(motores[i][2], LOW);
        delay(motorDelay);
        digitalWrite(motores[i][2], HIGH);
        delay(motorDelay);
        }*/
    }
    else {
        sensor = i2cHall.digitalRead(motores[i][3]);
       delay(200);
       Serial.println(sensor);
    
    if(sensor==1){
      for (int j = 0; j < 2000; j++) {
        pcf8574.digitalWrite(motores[i][2], LOW);
        delay(motorDelay);
        pcf8574.digitalWrite(motores[i][2], HIGH);
        delay(motorDelay);
        }
    }
    }
  }
  delay(1000);
    

}
