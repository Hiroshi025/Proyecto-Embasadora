#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

// Configuración de pines
const int sensorPin = 3;
const int ledPin = 4;

// Inicializar la LCD en dirección 0x27, 16 columnas x 2 filas
LiquidCrystal_I2C lcd(0x27, 16, 2);

int frascos = 0;
int cajas = 0;
bool estadoAnterior = LOW;

void setup() {
    pinMode(sensorPin, INPUT);
    pinMode(ledPin, OUTPUT);
    lcd.init();
    lcd.backlight();

    // Cargar el número de cajas almacenado en la EEPROM para mantener persistencia
    EEPROM.get(0, cajas);
    actualizarLCD();
}

void loop() {
    bool estadoActual = digitalRead(sensorPin);
    
    // Detecta cuando el sensor se activa (transición de LOW a HIGH)
    if (estadoActual == HIGH && estadoAnterior == LOW) {
        frascos++; // Incrementar el contador de frascos
        
        if (frascos >= 5) { // Si se han contado 5 frascos
            frascos = 0; // Reiniciar el contador de frascos
            cajas++; // Incrementar el contador de cajas
            EEPROM.put(0, cajas); // Guardar el número de cajas en EEPROM
            
            // Encender el LED al mismo tiempo que se actualiza el LCD
            digitalWrite(ledPin, HIGH);
            actualizarLCD(); // Actualizar la pantalla de inmediato
            delay(1000); // Mantener el LED encendido por 2 segundos
            digitalWrite(ledPin, LOW);
        } else {
            actualizarLCD(); // Actualizar la pantalla en cada detección de frasco
        }
    }
    estadoAnterior = estadoActual; // Guardar el estado anterior del sensor
}

// Función para actualizar la información en la pantalla LCD
void actualizarLCD() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Frascos: ");
    lcd.print(frascos);
    lcd.setCursor(0, 1);
    lcd.print("Num. Cajas: ");
    lcd.print(cajas);
}