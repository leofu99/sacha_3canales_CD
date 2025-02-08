#include <Wire.h>   // Incluye librería para interfaz I2C
#include "PCF8574.h" // Librería para el PCF8574

enum  MotorState {
  REST,
  DROP,
  WAIT,
  CLOSE
};

struct Motor {
  bool isI2c;
  int dir;
  int pull;
  int hall;
  unsigned long previousTime;
  unsigned long previousTimeSensor;
  bool toogle;
  bool isAtStartPoint;
  MotorState state;
  int count;

};
Motor motores[] = {
  {false, 4, 5, 0, 0, 0, false, false, CLOSE, 0},
  {false, 6, 7, 1, 0, 0, false, false, CLOSE, 0},
  {false, 8, 9, 2, 0, 0, false, false, CLOSE, 0},
  {true, 0, 1, 4, 0, 0, false, false, CLOSE, 0},
  {true, 2, 4, 5, 0, 0, false, false, CLOSE, 0},
  {true, 5, 6, 6, 0, 0, false, false, CLOSE, 0}
};
unsigned long MotorInterval = 2000;
unsigned long sensorInterval = 100000;
int waitSteps = 400;
int dropSteps = 200;

PCF8574 i2cSorter(0x25); // Dirección del PCF8574 de los motores
PCF8574 i2cHall(0x23); // Dirección del PCF8574 de los sensores Hall



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
    readSensors(currentTime, i);
  }
}


void moveMotor(unsigned long currentTime, int i) {

  if (currentTime - motores[i].previousTime >= MotorInterval) {

    motores[i].previousTime = currentTime;
    if (enableMotor(motores[i])){
    motores[i].toogle = !(motores[i].toogle);
      toogleMotor(i, motores[i].toogle);
    }
  }

}

void toogleMotor(int i, bool toogle) {
  if (motores[i].isI2c) {
    i2cSorter.digitalWrite(motores[i].pull, toogle ?  HIGH : LOW);
  }
  else {
    digitalWrite(motores[i].pull, toogle ?  HIGH : LOW);
  }
}

void readSensors(unsigned long currentTime, int i) {
  if (currentTime - motores[i].previousTimeSensor >= sensorInterval) {
    motores[i].previousTimeSensor = currentTime;
    // Leer el sensor Hall desde el PCF8574
    int sensor = i2cHall.digitalRead(motores[i].hall);
    // Desactivar el motor si el sensor detecta algo (HIGH)
    motores[i].isAtStartPoint = (sensor == LOW);
  }
}

boolean enableMotor(Motor &motor ) {
  boolean enable = false;
  switch (motor.state) {
    case REST:
      enable = false;
      break;
    case DROP:
      if (motor.count < dropSteps) {
        enable = true;
      }
      else {
        motor.state = WAIT;
        enable = false;
      }
      break;
    case WAIT:
      if (motor.count < waitSteps) {
        enable = false;
      }
      else {
        motor.state = CLOSE;
        enable = true;
      }
      break;
    case CLOSE:
      if (motor.isAtStartPoint) {
        enable = false;
        motor.state = REST;
      }
      else {
        enable = true;
      }
      break;
  }
  motor.count ++;
  return enable;
}
