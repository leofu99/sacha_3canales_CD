#include <Wire.h>

void setup() {
  Wire.begin();
  Serial.begin(9600);
  Serial.println("Escaneo I2C en progreso...");
}

void loop() {
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.print("Dispositivo encontrado en 0x");
      Serial.println(address, HEX);
      delay(500);
    }
  }
  delay(2000);
}
