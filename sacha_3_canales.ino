// librerías
#include <Wire.h>                 //libreria para comunicacion I2C
#include <Keypad_I2C.h>           //libreria para teclado matricial I2C
#include <LiquidCrystal_I2C.h>
#include <RTClib.h>   // incluye libreria para el manejo del modulo RTC
#undef PCF8574
#include "PCF8574.h" //i2c sensores hall
//#include "HX711.h" //celda de carga


//definiciones
// direcciones de i2c
#define I2CADDR     0x20       //teclado matricial
#define LCD_ADDRESS 0x27       //pantalla LCD
#define LCD_COLUMNS 20
#define LCD_ROWS    4


//variables para el control del motor principal

int targetVel = 300; // Velocidad objetivo
int mainMotor = 7; // Pin para la señal de pulso
unsigned long previousMainMotorTime = 0;
unsigned long mainMotorInterval = 5000; // Intervalo para el motor (espera inicial en us)
unsigned long previousRampTime = 0; // Tiempo anterior para la rampa
unsigned long rampInterval = 10; // Intervalo para actualizar la rampa (ms)
bool motorState = false; // toogle para el motor
bool activeMotor = true; // encender/apagar el motor
bool isStopping = false;// en proceso de parar el otor
int motorVelRange[2] = {300, 1400};

//variables para el manejo del teclado

const byte ROWS = 4;              // teclado de cuatro filas
const byte COLS = 4;              // teclado de cuatro columnas
//define los simboles para cada boton del teclado
char hexaKeys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};
byte rowPins[ROWS] = {6, 5, 4, 3}; // conecta las terminales fila al modulo I2C
byte colPins[COLS] = {2, 1, 0, 7}; // conecta las terminales columna al modulo I2C
Keypad_I2C customKeypad( makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS, I2CADDR); // crea el objeto para el teclado matricial
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS); // objeto para la pantalla
RTC_DS3231 rtc;     //objeto del tipo RTC
PCF8574 pcf8574(0x27);

void setup() {
  //verificación del funcionamiento de los módulos
   if (! rtc.begin()) {       // si falla la inicializacion del modulo
 //Serial.println("Modulo RTC no encontrado !");
// while (1); 
  }
  pinMode(mainMotor, OUTPUT);
  customKeypad.begin( );
  lcd.init();
  lcd.backlight();
  Serial.begin(9600);
  Serial.println("Listo para recibir comandos de velocidad.");
  lcd.setCursor(0, 0); // Establecer el cursor en la primera fila, primera columna
  lcd.print("Buenos días :)");
}

void loop() {
  checkKeyboard();
  handleSerialInput(); // Manejar entrada serial
  updateSpeedRamp();   // Ajustar la velocidad suavemente
  moveMainMotor();     // Control del motor
}

void moveMainMotor() {
  if (activeMotor == true) {
    unsigned long currentTime = micros();
    if (currentTime - previousMainMotorTime >= mainMotorInterval) {
      previousMainMotorTime = currentTime;
      motorState = !motorState; // Alterna el estado del motor
      digitalWrite(mainMotor, motorState ? HIGH : LOW);
    }
  }
}

void handleSerialInput() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n'); // Leer entrada serial
    int newTargetVel = input.toInt();           // Convertir a entero
    if (newTargetVel > motorVelRange[0]  && newTargetVel < motorVelRange[1]   ) {
      activeMotor = true;
      rampInterval = 25;
      targetVel = newTargetVel;                 // Actualizar velocidad objetivo
      Serial.print("Velocidad objetivo: ");
      Serial.println(targetVel);
    } else if (newTargetVel == 0) {
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
        if (isStopping) {
          isStopping = false;
          activeMotor = false;
        }
      }
    }
  }
}

void checkKeyboard() {
  char tecla_presionada = customKeypad.getKey();  // detecta la tecla presionada
  if (tecla_presionada != NO_KEY) {               //verifica si hubo una tecla presionada
    Serial.println(tecla_presionada);             //escribe la tecla presionada en puerto serie
  }

}
