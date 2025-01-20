#include <Wire.h>   // Incluye librería para interfaz I2C
#include "PCF8574.h" // Librería para el PCF8574

PCF8574 pcf8574(0x23); // Dirección del PCF8574

void setup() {
  Serial.begin(9600);

  // Configurar todos los pines del PCF8574 como entradas
  for (int i = 0; i < 8; i++) {
    if(i!=3){
    pcf8574.pinMode(i, INPUT);
    }

  }
//delay(5000);
 //pcf8574.begin(); // Iniciar comunicación I2C
  Serial.println("Listo para leer sensores.");
}

void loop() {
  for (int i = 0; i < 8; i++) {
    if(i != 3){
    delay(1000); 
    int sensorState = pcf8574.digitalRead(i); // Leer estado del pin

    Serial.print("Pin ");
    Serial.print(i);
    Serial.print(": "); 
    Serial.println(sensorState == LOW ? "ACTIVADO" : "DESACTIVADO");

    if (sensorState == LOW) { // Sensor detectado (activa en LOW)
      Serial.print("Sensor detectado en pin ");
      Serial.println(i);
    }
  }
  }
  delay(1000); // Pausa de 1 segundo
}
