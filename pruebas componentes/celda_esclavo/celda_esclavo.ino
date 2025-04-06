#include <Wire.h>
#include "HX711.h"

#define SLAVE_ADDRESS 1  
#define DT A0  
#define SCK A1  

HX711 celda;
float tara = 0;
float factorCalibracion = 1.0;
volatile float pesoActual = 0.0;  // 📌 Variable global que almacena el último peso leído

void setup() {
    Wire.begin(SLAVE_ADDRESS);
    Wire.onRequest(enviarPeso);

    Serial.begin(9600);
    celda.begin(DT, SCK);
    celda.set_scale();
    celda.tare();
    tara = celda.get_units(10);

    Serial.println("✅ Esclavo listo.");
}

void loop() {
    if (celda.is_ready()) {
        pesoActual = (celda.get_units(10) - tara) * (-1);
        Serial.print("📊 Peso actualizado: ");
        Serial.println(pesoActual);
    }
    delay(500);  // Reducir la frecuencia de lectura para estabilidad
}

void enviarPeso() {
    Serial.println("📡 Solicitud de peso recibida. Enviando...");

    Wire.write((byte*)&pesoActual, 4);
}
