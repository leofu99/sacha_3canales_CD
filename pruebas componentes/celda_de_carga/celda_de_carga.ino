#include "HX711.h"

#define NUM_CELDAS 2


// Pines de conexión para cada HX711
const int DT_PINS[NUM_CELDAS] = { A2, A0};
const int SCK_PINS[NUM_CELDAS] = {  A3, A1};


HX711 celdas[NUM_CELDAS];
float tara[NUM_CELDAS];     // Valores en reposo
float factor[NUM_CELDAS];   // Factores de calibración

void setup() {
    Serial.begin(9600);
    Serial.println("Iniciando calibración...");

    // Inicializa cada celda de carga
    for (int i = 0; i < NUM_CELDAS; i++) {
        celdas[i].begin(DT_PINS[i], SCK_PINS[i]);
        celdas[i].set_scale();  // Factor de escala por defecto
        celdas[i].tare();       // Tarear (ajustar el peso base)
        tara[i] = celdas[i].get_units(10);  // Guardar la tara inicial
    }

    Serial.println("Vamos a calibrar cada celda de carga por separado.");

    // Calibrar cada celda individualmente
    for (int i = 0; i < NUM_CELDAS; i++) {
        Serial.print("Coloca un peso en la celda ");
        Serial.print(i + 1);
        Serial.println(" y presiona Enter.");

        while (Serial.available() == 0) {}  // Espera a que el usuario presione Enter
        Serial.read();  // Limpia buffer

        Serial.print("Introduce el peso de referencia para la celda ");
        Serial.print(i + 1);
        Serial.println(" en gramos:");

        while (Serial.available() == 0) {}  // Espera a que el usuario escriba el peso
        float pesoReferencia = Serial.parseFloat();
        Serial.read();  // Limpia buffer

        float pesoLeido = celdas[i].get_units(10) - tara[i]; // Diferencia con la tara
        factor[i] = pesoReferencia / pesoLeido;  // Calcula el factor de conversión
        celdas[i].set_scale(factor[i]);  // Aplica la calibración

        Serial.print("Celda ");
        Serial.print(i + 1);
        Serial.println(" calibrada.");
        Serial.println(pesoLeido);
    }

    Serial.println("Calibración finalizada.");
}

void loop() {
    Serial.print("Pesos: ");
    for (int i = 0; i < NUM_CELDAS; i++) {
        float peso = (celdas[i].get_units(10) - tara[i])*(-1);  // Ajusta con la tara
        Serial.print(peso);
        Serial.print(" g | ");
        //Serial.print(tara[i]);
    }
    Serial.println();
    delay(500);


}
