#include <Wire.h>   // Incluye librería para interfaz I2C
#include "PCF8574.h" // Librería para el PCF8574

PCF8574 pcf8574(0x25); // Dirección del PCF8574

int targetVel = 300; // Velocidad objetivo
int motorPull = 0; // Pin para la señal de pulso
int motorDir = 1;
unsigned long previousMainMotorTime = 0;
unsigned long mainMotorInterval = 5000; // Intervalo para el motor (us)
unsigned long previousRampTime = 0; // Tiempo anterior para la rampa
 unsigned long rampInterval = 10; // Intervalo para actualizar la rampa (ms)
bool motorState = false; // toogle para el motor
bool isMoving = true;
bool isStopping = false;
int velMin = 300;
int velMax = 1400;

void setup() {
  Serial.begin(9600);

  // Configurar todos los pines del PCF8574 como entradas
  for (int i = 0; i < 8; i++) {
    if(i!=3){
    pcf8574.pinMode(i, OUTPUT);
    }

  }
  Serial.println("Listo para MOVER MOTORES.");
}

void loop() {

   handleSerialInput(); // Manejar entrada serial
  updateSpeedRamp();   // Ajustar la velocidad suavemente
  moveMainMotor();   

}

void moveMainMotor() {
  if(isMoving == true) {
  unsigned long currentTime = micros();
  if (currentTime - previousMainMotorTime >= mainMotorInterval) {
    previousMainMotorTime = currentTime;
    motorState = !motorState; // Alterna el estado del motor
    pcf8574.digitalWrite(motorPull, motorState ? HIGH : LOW);
  }
  }
}

void handleSerialInput() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n'); // Leer entrada serial
    int newTargetVel = input.toInt();           // Convertir a entero
    if (newTargetVel > velMin && newTargetVel <velMax   ) {
      isMoving = true;
      rampInterval = 25; 
      targetVel = newTargetVel;                 // Actualizar velocidad objetivo
      Serial.print("Velocidad objetivo: ");
      Serial.println(targetVel);
    } else if(newTargetVel==0){
      Serial.println("Deteniendo el motor");
      targetVel = 150;
      isStopping = true; 
      rampInterval = 2; 
      } else {
      Serial.println("Entrada no válida.");
    }
  }
}

void updateSpeedRamp() {
  unsigned long currentTime = millis();
  if (currentTime - previousRampTime >= rampInterval) {
    previousRampTime = currentTime;

    // Convertir velocidad objetivo a intervalo deseado
    unsigned long targetInterval = 1000000 / targetVel;

    // Ajustar suavemente hacia el intervalo objetivo
    if (mainMotorInterval > targetInterval) {
      mainMotorInterval -= 10; // Reducir el intervalo (acelera)
      if (mainMotorInterval < targetInterval) {
        mainMotorInterval = targetInterval; // Evitar sobrepasar
        
      }
    } else if (mainMotorInterval < targetInterval) {
      mainMotorInterval += 10; // Aumentar el intervalo (desacelera)
      if (mainMotorInterval > targetInterval) {
        mainMotorInterval = targetInterval; // Evitar sobrepasar
        if(isStopping){
          isStopping = false;
          isMoving = false;
          }
      }
    }
  }
}
