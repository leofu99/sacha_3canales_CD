/* tecnologias jpc
 *  programa para usar teclado matricial 4x4 con modulo PCF8574 y pantalla OLED
 *  circuito https://drive.google.com/file/d/1safDFtSTFCdoUiZfRS5aiq9BK9p3Zf_4/view?usp=sharing
 *  compatible con tarjeta Arduino Shield Omega, adquierela en http://www.tecnologiasjpc.com/
*/
        // alto de pantalla OLED en pixeles  

#define I2CADDR         0x20       // direccion para teclado matricial

#include <Wire.h>                 //libreria para comunicacion I2C
#include <Keypad_I2C.h>           //libreria para teclado matricial I2C
              //libreria para teclado matricial

const byte ROWS = 4;              // teclado de cuatro filas
const byte COLS = 4;              // teclado de cuatro columnas

//define los simboles para cada boton del teclado
char hexaKeys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};

byte rowPins[ROWS] = {6, 5, 4, 3}; // conecta las terminales fila al modulo I2C 
byte colPins[COLS] = {2, 1, 0, 7}; // conecta las terminales columna al modulo I2C 


// crea el objeto para el teclado matricial
Keypad_I2C customKeypad( makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS, I2CADDR); 

void setup(){
  Serial.begin(9600);                     //inicia comunicacion serial
  customKeypad.begin( );                  //inicializa el teclado matricial
  

}
  
void loop(){
  char tecla_presionada = customKeypad.getKey();  // detecta la tecla presionada 
  if (tecla_presionada != NO_KEY){                //verifica si hubo una tecla presionada
    Serial.println(tecla_presionada);             //escribe la tecla presionada en puerto serie
                         //ejecuta comandos previos sobre pantalla OLED
  }
}
