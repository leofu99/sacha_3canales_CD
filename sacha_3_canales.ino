#include <Wire.h>
#include "PCF8574.h" // Librería para el expansor I2C
//comentario
// Configuración del PCF8574
PCF8574 pcf8574(0x27);

// Pines
int mainMotor = 7;  // Pin para la señal de pulso
int sensorPin = 0;  // Pin del sensor en el PCF8574

// Variables del motor
unsigned long previousMainMotorTime = 0;
int motorInterval = 1000;           // Intervalo inicial (µs)
const int maxSpeed = 1000;           // Velocidad máxima (µs)
const int minSpeed = 2000;          // Velocidad mínima (µs)
int rampStep = 10;                  // Incremento/decremento de la rampa
bool motorState = false;            // Estado del motor (ON/OFF)
bool motorRunning = true;           // Si el motor está en marcha

// Variables del sensor
bool sensorState = false;

void setup() {
  pinMode(mainMotor, OUTPUT);       // Configurar el pin del motor como salida
  pcf8574.pinMode(sensorPin, INPUT); // Configurar el pin del sensor como entrada
  Serial.begin(9600);               // Inicializar la comunicación serial
}

void loop() {
  readSensor();         // Leer el estado del sensor
  rampMotorControl();   // Controlar el motor con la rampa
}

void rampMotorControl() {
  // Control del motor con rampa de velocidad
  if (!motorRunning) {
    // Si el motor está detenido, asegurarse de que no genere pulsos
    digitalWrite(mainMotor, LOW);
    return;
  }

  unsigned long currentTime = micros();
  if (currentTime - previousMainMotorTime >= motorInterval) {
    previousMainMotorTime = currentTime;

    // Generar señal de pulso para el motor
    motorState = !motorState; // Alternar el estado del motor
    digitalWrite(mainMotor, motorState ? HIGH : LOW);

    // Ajustar la velocidad gradualmente
    if (sensorState) {
      // Si el sensor está activado, pausa el motor (velocidad al mínimo)
      motorInterval = minSpeed;
    } else {
      // Cambiar la velocidad progresivamente hacia el objetivo
      if (motorInterval > maxSpeed) {
        motorInterval -= rampStep; // Aumentar velocidad
      } else if (motorInterval < minSpeed) {
        motorInterval += rampStep; // Reducir velocidad
      }
    }
  }
}

void readSensor() {
  // Leer el sensor con un intervalo de tiempo razonable (100 ms)
  static unsigned long previousSensorTime = 0;
  unsigned long currentTime = millis();
  if (currentTime - previousSensorTime >= 100) {
    previousSensorTime = currentTime;

    // Leer el estado del sensor desde el PCF8574
    sensorState = pcf8574.digitalRead(sensorPin);
    if (sensorState) {
      // Si el sensor está activado, detener el motor
      motorRunning = false;
      Serial.println("Sensor activado: Motor detenido.");
    } else {
      // Si el sensor está desactivado, permitir el funcionamiento del motor
      motorRunning = true;
      Serial.println("Sensor desactivado: Motor en funcionamiento.");
    }
  }
}
