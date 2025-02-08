#include <Wire.h>   // Incluye librería para interfaz I2C
#include "PCF8574.h" // Librería para el PCF8574

struct Motor {
  bool isI2c;       // 0 = Directo, 1 = I2C
  int dir;
  int pull;
  int hall;
  unsigned long previousTime;
  unsigned long previousTimeSensor;
  bool state;    // Booleano adicional
  bool enable;
};
Motor motores[] = {
  {false, 4, 5, 0,0,0,false, true},  // Motor 1: Directo, pines 4 y 5
  {false, 6, 7, 1,0,0, false,true},  // Motor 2: Directo, pines 6 y 7
  {false, 8, 9, 2,0,0,false,true},  // Motor 3: Directo, pines 8 y 9
  {true, 0, 1, 4,0,0,false,true},  // Motor 4: I2C, pines 0 y 1 en PCF8574
  {true, 2, 4, 5, 0,0,false,true},  // Motor 5: I2C, pines 2 y 4 en PCF8574
  {true, 5, 6, 6,0,0,false,true}   // Motor 6: I2C, pines 5 y 6 en PCF8574
};
unsigned long MotorInterval = 2000;
unsigned long sensorInterval = 100000;

PCF8574 i2cSorter(0x25); // Dirección del PCF8574
PCF8574 i2cHall(0x23); // Dirección del PCF8574



void setup() {
  Serial.begin(9600);
  for (int i = 0; i < 6; i++) {
    if (motores[i].isI2c) {  // Si es directo
      i2cSorter.pinMode(motores[i].dir, OUTPUT);
      i2cSorter.pinMode(motores[i].pull, OUTPUT);
      i2cSorter.digitalWrite(motores[i].dir, LOW);

      
    } else {

      pinMode(motores[i].dir, OUTPUT);
      pinMode(motores[i].pull, OUTPUT);
      digitalWrite(motores[i].dir, LOW);

    }
    i2cHall.pinMode(motores[i].hall, INPUT);
  }
}

void loop() {
  unsigned long currentTime = micros();
  for (int i = 0; i < 6; i++) {
    moveMotor(currentTime, i);
    readSensors(currentTime,i);  
    }
}


void moveMotor(unsigned long currentTime, int i){
  if (currentTime - motores[i].previousTime >= MotorInterval){
    motores[i].previousTime = currentTime;
    motores[i].state = !(motores[i].state);
    toogleMotor(i, motores[i].state);
    }
}

void toogleMotor(int i, bool state) {
  
  if(motores[i].enable){
      if(motores[i].isI2c) {
    i2cSorter.digitalWrite(motores[i].pull, state ?  HIGH : LOW);
    } 
  else {
    digitalWrite(motores[i].pull, state ?  HIGH : LOW);
  }}


}

void readSensors(unsigned long currentTime, int i) {
    if (currentTime - motores[i].previousTimeSensor >= sensorInterval) {
        motores[i].previousTimeSensor = currentTime;
        
        // Leer el sensor Hall desde el PCF8574
        int sensor = i2cHall.digitalRead(motores[i].hall);

        // Desactivar el motor si el sensor detecta algo (HIGH)
        motores[i].enable = (sensor == LOW);
    }
}
  
