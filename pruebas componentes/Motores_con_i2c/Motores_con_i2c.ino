#include <Wire.h>   // Incluye librería para interfaz I2C
#include "PCF8574.h" // Librería para el PCF8574
#include <TimerOne.h> // Librería para temporizadores

volatile bool readSensorsFlag = false; // Bandera para la ISR
int motor = 0;  // Variable para el motor
int direccion = 0;  // Variable para la dirección

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
  bool finishClose;
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
unsigned long sensorInterval = 5000;
int waitSteps = 300;
int dropSteps = 100;

PCF8574 i2cSorter(0x20); // Dirección del PCF8574 de los motores
PCF8574 i2cHall(0x27); // Dirección del PCF8574 de los sensores Hall



void setup() {
  //Wire.begin();
   Wire.setClock(100000); 
  Serial.begin(9600);
  //Serial.println("hola");
  for (int i = 0; i < 6; i++) {
     i2cHall.pinMode(motores[i].hall, INPUT);
    if (motores[i].isI2c) {  // Si es directo
      i2cSorter.pinMode(motores[i].dir, OUTPUT);
      i2cSorter.pinMode(motores[i].pull, OUTPUT);
      i2cSorter.digitalWrite(motores[i].dir, HIGH);
    } else {
      pinMode(motores[i].dir, OUTPUT);
      pinMode(motores[i].pull, OUTPUT);
      digitalWrite(motores[i].dir, LOW);
    }
   
  }

  //interrupción
  Timer1.initialize(40000);  // 50ms
  Timer1.attachInterrupt(timerISR);
}

void loop() {

  
    if (readSensorsFlag) {
    readSensorsFlag = false; // Reiniciar bandera
    readAllSensors();
  }
  readMotorSet();
  unsigned long currentTime = micros();
  for (int i = 0; i < 6; i++) {
    moveMotor(currentTime, i);



  }
}


void moveMotor(unsigned long currentTime, int i) {

  if (currentTime - motores[i].previousTime >= MotorInterval) {
     if (motores[i].state == WAIT || motores[i].state == DROP) {
        motores[i].count ++;
      }
    motores[i].previousTime = currentTime;
    if (enableMotor(motores[i])) {
      motores[i].toogle = !(motores[i].toogle);
      toogleMotor(i, motores[i].toogle);
    } 
  }

}

void toogleMotor(int i, bool toogle) {
  if (motores[i].isI2c) {
    i2cSorter.digitalWrite(motores[i].pull, toogle ?  HIGH : LOW);
    //delayMicroseconds(100); // Para evitar saturación del bus
  }
  else {
    digitalWrite(motores[i].pull, toogle ?  HIGH : LOW);
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
        Serial.println("estado drop -> wait ");

        Serial.println(motor.count);
        motor.state = WAIT;
        enable = false;
      }
      break;
    case WAIT:

      if (motor.count < waitSteps) {
        enable = false;
      }
      else {
        Serial.println("estado wait -> close");
    
        Serial.println(motor.count);
        motor.state = CLOSE;
        motor.count = 0; // Reiniciar el contador
        enable = true;
      }
      break;
    case CLOSE:
      if (motor.finishClose) {
        Serial.println("estado close -> rest: ");
   
        Serial.println(motor.count);
        enable = false;
        motor.state = REST;
        motor.finishClose = false;
        
 
      }
      else {
        enable = true;
      }
      break;
  }


  return enable;


}


void readMotorSet() {
  if (Serial.available() > 0) {
    // Lee la línea de texto
    String input = Serial.readStringUntil('\n');  // Lee hasta el salto de línea

    // Divide la cadena por el espacio o coma, dependiendo de cómo se envíen los datos
    int separatorIndex = input.indexOf(',');  // Busca la coma (puedes cambiar el delimitador)

    if (separatorIndex != -1) {
      // Si encontró la coma, divide la cadena en dos partes
      motor = input.substring(0, separatorIndex).toInt();
      direccion = input.substring(separatorIndex + 1).toInt();
    } else {
      // Si no encontró coma, se asume que hay un espacio
      motor = input.substring(0, input.indexOf(' ')).toInt();
      direccion = input.substring(input.indexOf(' ') + 1).toInt();
    }

      drop(motor, direccion);
  }
}

void drop(int motor, int dir) {
  if (dir != 0 && dir != 1) {
    return;  // Sale de la función si la dirección no es 0 o 1
  }
  motores[motor].state = DROP;
  
  motores[motor].count = 0;
  if (motores[motor].isI2c) {
    i2cSorter.digitalWrite(motores[motor].dir, !dir);
  } else {
    digitalWrite(motores[motor].dir, dir);
  }

}

void timerISR() {
  readSensorsFlag = true; // Activar la bandera
}

void readAllSensors() {
  for (int i = 0; i < 6; i++) {
    if(motores[i].state == CLOSE ){
    int sensor = i2cHall.digitalRead(motores[i].hall);
    if (sensor == HIGH || sensor == LOW) {  // Evita valores incorrectos  
        motores[i].finishClose = (sensor == LOW);
     
}
    }
  }
}
