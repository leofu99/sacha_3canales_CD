#include <Wire.h>
#include "HX711.h"

#define DT A0
#define SCK A1

// {-1879.012, -1747.81,  -1816.27 }
HX711 celda;
float tara = 0;
float factorCalibracion = -1816.27;
volatile float pesoActual = 0.0;  // 📌 Variable global que almacena el último peso leído

void setup() {
  // pines de los motores
  for (int i = 4; i < 10; i++) {
    pinMode(i, OUTPUT);
    digitalWrite(i, LOW);
  }

  Serial.begin(9600);
  // Unimos este dispositivo al bus I2C con dirección 1
  Wire.begin(1);
  // Registramos el evento al recibir datos
  Wire.onReceive(receiveData);
  Wire.onRequest(enviarPeso);

//INIT CELDA
  celda.begin(DT, SCK);
  celda.set_scale(factorCalibracion);
  celda.tare();
  tara = celda.get_units(10);
  Serial.println("✅ Esclavo listo.");


  // Iniciamos el monitor serie para monitorear la comunicación
}

void loop() {
  updateWeight();
}

void receiveData(int bytes) {
  // Leemos el pin y el estado que llegaron del maestro
  byte pin = Wire.read();     // El primer byte es el pin
  byte estado = Wire.read();  // El segundo byte es el estado (HIGH o LOW)
  // Aplicamos el estado al pin correspondiente
  digitalWrite(pin, estado ? HIGH : LOW);
}

void updateWeight(){
      if (celda.is_ready()) {
        pesoActual = (celda.get_units(10) - tara);
        Serial.print("📊 Peso actualizado: ");
        Serial.println(pesoActual);
    }
   // delay(300);  // Reducir la frecuencia de lectura para estabilidad
}

void enviarPeso() {
    //Serial.println("📡 Solicitud de peso recibida. Enviando...");
    Wire.write((byte*)&pesoActual, 4);
}
