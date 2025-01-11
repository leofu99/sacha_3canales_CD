int DIR = 6; // Pin de dirección del motor
int EN = 5; // Pin Enable (no es imprescindible conectarlo)
int vel = 1000;
int pot = 0;
int auxPot = 0;
int mainMotor = 7; // Pin para la señal de pulso

int pin = 0;
int sensor =0;
unsigned long previousMainMotorTime = 0;
const unsigned long mainMotorInterval = 2000; // Intervalo para el motor (us)
bool motorState = false;

void setup(){
    pinMode(mainMotor, OUTPUT);
    pinMode(DIR, OUTPUT);
    pinMode(EN, OUTPUT);
    digitalWrite(EN, HIGH);
    Serial.begin(9600);

}

void loop(){

  moveMainMotor();
}

void moveMainMotor() {
    // Control del motor
  unsigned long currentTime = micros();
  if (currentTime - previousMainMotorTime >= mainMotorInterval) {
    previousMainMotorTime = currentTime;
    motorState = !motorState; // Alterna el estado del motor
    digitalWrite(mainMotor, motorState ? HIGH : LOW);
  }
}
