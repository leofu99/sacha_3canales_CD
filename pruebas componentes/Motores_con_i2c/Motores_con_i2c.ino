#include <Wire.h>      // Incluye librería para interfaz I2C
#include "PCF8574.h"   // Librería para el PCF8574
#include <TimerOne.h>  // Librería para temporizadores
#include "HX711.h"     //celda de carga

#define NUM_CELDAS 2  // En el maestro hay 2 celdas
#define PROMEDIO_LECTURAS 10
//celdas de carga
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
float factor[NUM_CELDAS] = { -1879.012, -1747.81 };

long lecturas[NUM_CELDAS][PROMEDIO_LECTURAS];
int indices[NUM_CELDAS] = { 0, 0 };
float pesos[NUM_CELDAS+1];

volatile bool readSensorsFlag = false;  // Bandera para la ISR
volatile bool readCellFlag = false;
int cellReadDivider = 0;
//variables para guardar ordenes para el motor desde el serial.
int motor = 0;      // Variable para el motor
int direccion = 0;  // Variable para la dirección
// fin celdas


enum MotorState {
  REST,
  DROP,
  WAIT,
  CLOSE
};

//---------------------------------------variables y estructuras del código gral------------------------------
struct Channel {
  int cell;
  int motor[2];
  int value[2];
};

struct Tray {
  String label;
  float minValue;
  float maxValue;
};

Tray trayList[] = {
  { "B", 46, 53 },
  { "A", 53, 60 },
  { "AA", 60, 67 },
  { "X", 67, 78 },
  { "J", 78, 99 },
  { "C", 35, 46 }
};

int directionId[2][2] = {
  { 3, 4 },
  { 1, 2 }

};

Channel chanelList[] = {
  { 0, { 0, 3 }, { 0, 0 } },
  { 1, { 1, 4 }, { 0, 0 } },
  { 2, { 2, 5 }, { 0, 0 } }
};

//-----------------------------------------fin gral-.-.........................................
struct Master {
  int pull;
  int targetVel;
  unsigned long previousMainMotorTime;
  unsigned long mainMotorInterval;  // Intervalo para el motor (us)
  unsigned long previousRampTime;   // Tiempo anterior para la rampa
  unsigned long rampInterval;       // Intervalo para actualizar la rampa (ms)
  bool motorState;
  bool isMoving;
  bool isStopping;
  int velMin;
  int velMax;
  int state;
  bool changeState;
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
  int centralPosition;
};
Motor motores[] = {
  { false, 4, 5, 0, 0, 0, false, false, CLOSE, 0, 7 },
  { false, 6, 7, 1, 0, 0, false, false, CLOSE, 0, 12 },
  { false, 8, 9, 2, 0, 0, false, false, CLOSE, 0, 9 },
  { true, 4, 5, 4, 0, 0, false, false, CLOSE, 0, 3 },
  { true, 6, 7, 5, 0, 0, false, false, CLOSE, 0, 16 },
  { true, 8, 9, 6, 0, 0, false, false, CLOSE, 0, 8 }
};

int masterHall[] = { 4, 5, 6, 7 };
int velList[] = { 500, 500, 500, 500 };

Master master = { 2, 300, 0, 5000, 0, 10, false, false, false, 300, 1700, 0, false };


unsigned long MotorInterval = 2000;
unsigned long sensorInterval = 5000;
int waitSteps = 100;
int dropSteps = 50;


PCF8574 i2cSorter(0x23);           // Dirección del PCF8574 de los motores
PCF8574 i2cHall(0x27);             // Dirección del PCF8574 de los sensores Hall
PCF8574 i2cCycleController(0x21);  // direccion del PCF8574 de los sensores del motor ppal



void setup() {



  Wire.begin();
  Wire.setClock(400000);
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

  pinMode(master.pull, OUTPUT);

  //interrupción
  Timer1.initialize(40000);  // 50ms
  Timer1.attachInterrupt(timerISR);
  // calibrateMotors();
  initI2cControllerInputs();
  //updateMasterVel(301);
  checkComponents();
  initLoadcells();
}

void loop() {

  // handleSerialInput(); // Manejar entrada serial
  updateSpeedRamp();  // Ajustar la velocidad suavemente
  moveMainMotor();    // Control del motor


  if (readSensorsFlag) {
    readSensorsFlag = false;  // Reiniciar bandera
    readAllSensors();
  }
  if (readCellFlag) {
    leerCeldas();
    readCellFlag = false;
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
      motores[i].count++;
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
    Wire.beginTransmission(1);
    // Enviamos el pin a controlar
    Wire.write(motores[i].pull);
    // Enviamos el estado (HIGH o LOW)
    Wire.write(toogle);
    // Paramos la transmisión
    Wire.endTransmission();
    //i2cSorter.digitalWrite(motores[i].pull, toogle ? HIGH : LOW);
    //delayMicroseconds(100); // Para evitar saturación del bus
  } else {
    digitalWrite(motores[i].pull, toogle ? HIGH : LOW);
  }
}






boolean enableMotor(Motor &motor) {
  boolean enable = false;
  switch (motor.state) {
    case REST:
      enable = false;
      break;
    case DROP:

      if (motor.count < dropSteps) {
        enable = true;
      } else {
        //Serial.println("estado drop -> wait ");

        //Serial.println(motor.count);
        motor.state = WAIT;
        enable = false;
      }
      break;
    case WAIT:

      if (motor.count < waitSteps) {
        enable = false;
      } else {
        // Serial.println("estado wait -> close");

        // Serial.println(motor.count);
        motor.state = CLOSE;
        motor.count = 0;  // Reiniciar el contador
        enable = true;
      }
      break;
    case CLOSE:
      if (motor.finishClose) {
        // Serial.println("estado close -> rest: ");

        if (motor.count < motor.centralPosition) {
          motor.count++;
          enable = true;  // Sigue moviéndose hasta el punto central
        } else {
          // Serial.println(motor.count);
          enable = false;
          motor.state = REST;
          motor.finishClose = false;
          motor.count = 0;
        }




      } else {
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
  readSensorsFlag = true;  // Activar la bandera

  if (++cellReadDivider >= 3) {  // Cada 2 interrupciones = 100ms si ISR es cada 50ms
    readCellFlag = true;
    cellReadDivider = 0;
  }
}

void readAllSensors() {
  for (int i = 0; i < 6; i++) {
    if (motores[i].state == CLOSE) {
      int sensor = i2cHall.digitalRead(motores[i].hall);
      if (sensor == HIGH || sensor == LOW) {  // Evita valores incorrectos
        motores[i].finishClose = (sensor == LOW);
      }
    }
  }

  configMasterState(master.state);
}

//finciones del motor ppal

void moveMainMotor() {
  if (master.isMoving == true) {
    unsigned long currentTime = micros();
    if (currentTime - master.previousMainMotorTime >= master.mainMotorInterval) {
      master.previousMainMotorTime = currentTime;
      master.motorState = !master.motorState;  // Alterna el estado del motor
      digitalWrite(master.pull, master.motorState ? HIGH : LOW);
    }
  }
}

void updateMasterVel(int newTargetVel) {


  // Convertir a entero
  if (newTargetVel > master.velMin && newTargetVel < master.velMax) {
    master.isMoving = true;
    master.rampInterval = 25;
    master.targetVel = newTargetVel;  // Actualizar velocidad objetivo
    //Serial.print("Velocidad objetivo: ");
    //Serial.println(master.targetVel);
  } else if (newTargetVel == 0) {
    //Serial.println("Deteniendo el motor");
    master.targetVel = 150;
    master.isStopping = true;
    master.rampInterval = 2;
  } else {
    //Serial.println("Entrada no válida.");
  }
}

void updateSpeedRamp() {
  unsigned long currentTime = millis();
  if (currentTime - master.previousRampTime >= master.rampInterval) {
    master.previousRampTime = currentTime;

    // Convertir velocidad objetivo a intervalo deseado
    unsigned long targetInterval = 1000000 / master.targetVel;

    // Ajustar suavemente hacia el intervalo objetivo
    if (master.mainMotorInterval > targetInterval) {
      master.mainMotorInterval -= 10;  // Reducir el intervalo (acelera)
      if (master.mainMotorInterval < targetInterval) {
        master.mainMotorInterval = targetInterval;  // Evitar sobrepasar
      }
    } else if (master.mainMotorInterval < targetInterval) {
      master.mainMotorInterval += 10;  // Aumentar el intervalo (desacelera)
      if (master.mainMotorInterval > targetInterval) {
        master.mainMotorInterval = targetInterval;  // Evitar sobrepasar
        if (master.isStopping) {
          master.isStopping = false;
          master.isMoving = false;
        }
      }
    }
  }
}

void calibrateMotors() {
  for (int i = 0; i < 6; i++) {
    int stepCount = 0;
    bool sensorDetected = false;  // Para saber cuándo deja de detectar

    // Configura la dirección del motor
    if (motores[i].isI2c) {
      i2cSorter.digitalWrite(motores[i].dir, HIGH);
    } else {
      digitalWrite(motores[i].dir, HIGH);
    }

    //Serial.print("Motor ");
    // Serial.print(i);
    // Serial.println(" comenzando calibración...");

    // Bucle para dar 2 vueltas completas (400 pasos)
    for (int j = 0; j < 200; j++) {
      toogleMotor(i, true);  // Avanza un paso
      delay(10);
      toogleMotor(i, false);  // Detiene momentáneamente el motor
      delay(10);

      // Lee el sensor
      if (i2cHall.digitalRead(motores[i].hall) == LOW) {
        stepCount++;
      }
    }

    //Serial.print("Motor ");
    //Serial.print(i);
    //Serial.print(" terminó calibración con ");
    //Serial.print(stepCount);
    //Serial.println(" pasos totales.");
  }
}

void initI2cControllerInputs() {
  for (int i = 0; i < 8; i++) {
    if (i != 3) {
      i2cCycleController.pinMode(i, INPUT);
    }
  }
}

void configMasterState(int state) {
  int sensorValue = i2cCycleController.digitalRead(masterHall[state]);
  if (sensorValue == HIGH || sensorValue == LOW) {
    if (sensorValue == LOW) {
      master.state = master.state < 1 ? master.state + 1 : 0;
      //Serial.print("cambio al  estado: ");
      //Serial.println(master.state);
      updateMasterVel(velList[master.state]);

      if (master.state == 1) {
        // for (int i = 0; i < 6; i++) {
        //   motores[i].state = DROP;
        //   motores[i].count = 0;
        // }
        updateChannel();

      }
      else {
        readWeight();
      }
      Serial.print("status: ");
      Serial.println(master.state);

      
      //checkMotors();
    }
  }
}

void checkComponents() {
  checkMotors();
}

void checkMotors() {
  bool ready = false;
  unsigned long sensorTime = micros();
  ;
  while (!ready) {
    unsigned long currentTime = micros();
    if (readSensorsFlag && currentTime - sensorTime > 2000000) {
      readSensorsFlag = false;  // Reiniciar bandera
      readAllSensors();
    }
    //unsigned long currentTime = micros();
    for (int i = 0; i < 6; i++) {
      moveMotor(currentTime, i);
    }
    ready = areAllInRest();
  }
  updateMasterVel(500);
}

bool areAllInRest() {
  for (int i = 0; i < 6; i++) {
    if (motores[i].state != REST) {
      return false;
    }
  }
  return true;
}


//-------------------------------------celdas de carga---------------
void readWeight() {


  float peso;
  //Serial.println("Solicitando peso al esclavo...");
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
    //Serial.print("✅ Peso recibido: ");
    //Serial.println(peso);
  } else {
    Serial.println("❌ Error: No se recibieron 4 bytes.");
  }


  // Mostrar resultados
  Serial.print("Pesos -> 1: ");
  Serial.print(pesos[0]);
  Serial.print(" g | 2: ");
  Serial.print(pesos[1]);
  Serial.print(" g | 3: ");
  Serial.println(peso);
  pesos[2]= peso;

}

void initLoadcells() {
  for (int i = 0; i < NUM_CELDAS; i++) {
    celdas[i].begin(DT_PINS[i], SCK_PINS[i]);
    celdas[i].set_scale(factor[i]);
    celdas[i].tare();
    tara[i] = celdas[i].get_units(10);
  }

  Serial.println("Celdas listas...");
}

// Esta función se ejecuta automáticamente cada 50 ms
void leerCeldas() {
  // Serial.println("leyendo...");
  for (int i = 0; i < NUM_CELDAS; i++) {
    lecturas[i][indices[i]] = celdas[i].get_units(1);
    indices[i] = (indices[i] + 1) % PROMEDIO_LECTURAS;

    // Calcular promedio y restar tara
    long suma = 0;
    for (int j = 0; j < PROMEDIO_LECTURAS; j++) {
      suma += lecturas[i][j];
    }
    pesos[i] = ((float)suma / PROMEDIO_LECTURAS) - tara[i];
  }
}


//--------------------------------CLASIFICACION-------------------------------------------

int findTray(float weight) {
  for (int i = 0; i < 4; i++) {
    if (weight >= trayList[i].minValue && weight < trayList[i].maxValue) {
      return i;
    }
  }
  return 4;
}

int encontrarIndice(int valor, int arreglo[2]) {
  if (arreglo[0] == valor) return 0;
  if (arreglo[1] == valor) return 1;
  return -1;
}

void updateChannel() {
  int dir = 0;
  int motor = 0;
  for (int i = 0; i < 3; i++) {
    chanelList[i].value[1] = chanelList[i].value[0];
    chanelList[i].value[0] = findTray(pesos[i]);
    Serial.println(chanelList[i].value[0]);


    for (int j = 0; j < 2; j++) {
      dir = encontrarIndice(chanelList[i].value[j], directionId[j]);
      motor = chanelList[i].motor[j];

      if (dir != -1) {

        if(motores[motor].isI2c){
        Wire.beginTransmission(1);
        Wire.write(motores[motor].dir);
        Wire.write(dir == 1);
        Wire.endTransmission();
        } else {
          digitalWrite(motores[motor].dir, dir==0 ? HIGH : LOW);
        }


        motores[motor].state = DROP;
        motores[motor].count = 0;
      }
    }
  }
}

// -------------------------------------- fin clasificación ------------------------------------------
