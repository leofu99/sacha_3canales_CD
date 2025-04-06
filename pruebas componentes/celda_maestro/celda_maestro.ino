#include <Wire.h>
#include "HX711.h"

#define SLAVE_ADDRESS 1  // Dirección del esclavo

#define NUM_CELDAS 2  // En el maestro hay 2 celdas

// Pines de conexión para cada HX711 en el Maestro
const int DT_PINS[NUM_CELDAS] = {
  A0,
  A2,
};
const int SCK_PINS[NUM_CELDAS] = {
  A1,
  A3,
};

HX711 celdas[NUM_CELDAS];
float tara[NUM_CELDAS];
float factor[NUM_CELDAS];

void setup() {
  Wire.begin();
  Serial.begin(9600);
  Serial.println("Iniciando calibración...");

  for (int i = 0; i < NUM_CELDAS; i++) {
    celdas[i].begin(DT_PINS[i], SCK_PINS[i]);
    celdas[i].set_scale();
    celdas[i].tare();
    tara[i] = celdas[i].get_units(10);
  }

  Serial.println("Calibrando las celdas...");
  for (int i = 0; i < NUM_CELDAS; i++) {
    Serial.print("Coloca un peso en la celda ");
    Serial.print(i + 1);
    Serial.println(" y presiona Enter.");

    while (Serial.available() == 0) {}
    Serial.read();

    Serial.print("Introduce el peso de referencia para la celda ");
    Serial.print(i + 1);
    Serial.println(" en gramos:");

    while (Serial.available() == 0) {}
    float pesoReferencia = Serial.parseFloat();
    Serial.read();

    float pesoLeido = celdas[i].get_units(10) - tara[i];
    factor[i] = pesoReferencia / pesoLeido;
    celdas[i].set_scale(factor[i]);

    Serial.print("Celda ");
    Serial.print(i + 1);
    Serial.println(" calibrada.");
  }

  Serial.println("Calibración finalizada.");
}

void loop() {
  Serial.println("Entrando en loop...");
  float pesos[NUM_CELDAS];

  for (int i = 0; i < NUM_CELDAS; i++) {
    pesos[i] = (celdas[i].get_units(10) - tara[i]) * (-1);
  }
  float peso;

  Serial.println("Solicitando peso al esclavo...");
  Wire.requestFrom(1, 4);
    if (Wire.available() == 4) {
        byte pesoBytes[4];

        // Leer los 4 bytes
        for (int i = 0; i < 4; i++) {
            pesoBytes[i] = Wire.read();
        }

        // Convertir los bytes en un float
        //float peso;
        memcpy(&peso, pesoBytes, 4);

        Serial.print("✅ Peso recibido: ");
        Serial.println(peso);
    } else {
        Serial.println("❌ Error: No se recibieron 4 bytes.");
    }

    delay(1000);

  // Mostrar resultados
  Serial.print("Pesos -> 1: ");
  Serial.print(pesos[0]);
  Serial.print(" g | 2: ");
  Serial.print(pesos[1]);
  Serial.print(" g | 3 (Esclavo): ");
  Serial.println(peso);

  delay(1000);
}
