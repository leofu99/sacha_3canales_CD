#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Configuración de la dirección I2C y el tamaño de la LCD
#define LCD_ADDRESS 0x27
#define LCD_COLUMNS 20
#define LCD_ROWS 4

// Inicializar el objeto de la LCD
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS);

void setup() {
  // Inicializar la LCD
  lcd.init();
  // Encender la luz de fondo
  lcd.backlight();
  
  // Mostrar el mensaje en la LCD
  lcd.setCursor(0, 0); // Establecer el cursor en la primera fila, primera columna
  lcd.print("Hello, World!");
}

void loop() {
  // No se necesita código en el loop para este ejemplo
}
