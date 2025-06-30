/*
 * Sistema Automatizado de Envasado con Brazo Robótico (Versión Optimizada)
 */
#include <avr/pgmspace.h>
#include "HX711.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <Adafruit_PWMServoDriver.h>

// *********************** DEFINICIÓN DE PINES ***********************
const byte DT = 3, CLK = 2, modo = 7, tara = 6, banda = 8, led = 4;
const byte electro1 = 9, electro2 = 10, electro3 = 5;
const byte sensor1 = 11, sensor2 = 12, led2 = 13;
const byte sensor3 = A0, ledBloqueo = A1, ledSellado = A2;

const bool DEBUG_MODE = true;
unsigned long lastDebugTime = 0;

// *********************** BRAZO ROBÓTICO ***********************
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Datos en PROGMEM para ahorrar RAM
const PROGMEM uint16_t movimientos[][6] = {
  {150, 333, 290, 555, 456, 375}, // Posición HOME
  {400, 350, 450, 500, 300, 375}, // Paso 1
  {350, 400, 400, 450, 350, 375}, // Paso 2
  {300, 450, 350, 400, 400, 375}, // Paso 3
  {350, 400, 400, 450, 350, 375}, // Paso 4
  {432, 234, 599, 200, 300, 50}   // Paso 5
};

const PROGMEM uint16_t servoMin[6] = {150, 150, 150, 150, 150, 150};
const PROGMEM uint16_t servoMax[6] = {600, 600, 600, 600, 600, 600};
uint16_t servoPos[6] = {200, 200, 170, 550, 400, 333};

byte movimientoActual = 0;
bool brazoActivo = false;
unsigned long tiempoMovimientoBrazo = 0;

// *********************** VARIABLES DEL SISTEMA ***********************
volatile byte frascos = 0, cajas = 0; // Usamos byte (0-255) en lugar de int
bool estadoAnteriorSensor2 = HIGH;
unsigned long tiempoPiston = 0, tiempoSellador = 0;

// *********************** MÁQUINA DE ESTADOS ***********************
enum EstadoSistema : byte { // Usamos byte para el enum
  ESPERANDO_FRASCO, LLENANDO_FRASCO, EXPULSANDO_FRASCO,
  FRASCO_EN_BANDA, ESPERANDO_SELLADO, SELLANDO_FRASCO
};
EstadoSistema estadoActual = ESPERANDO_FRASCO;

// *********************** BALANZA ***********************
const PROGMEM uint16_t peso_conocido[5] = {500, 1000, 3000, 5000, 10000};
long escala;
const float limite_peso = 2000, peso_minimo = 50;

// *********************** DISPLAYS ***********************
LiquidCrystal_I2C lcd(0x26, 16, 2);
LiquidCrystal_I2C lcd2(0x27, 16, 2);
HX711 balanza;

// *********************** FUNCIONES OPTIMIZADAS ***********************
void moverServo(byte servo, uint16_t angulo) {
  angulo = constrain(angulo, pgm_read_word(&servoMin[servo]), pgm_read_word(&servoMax[servo]));
  servoPos[servo] = angulo;
  pwm.setPWM(servo, 0, angulo);
  
  if(DEBUG_MODE) {
    Serial.print(F("Servo ")); Serial.print(servo);
    Serial.print(F(" movido a: ")); Serial.println(angulo);
  }
}

void ejecutarSecuenciaBrazo() {
  if(brazoActivo && millis() - tiempoMovimientoBrazo >= 2000) {
    tiempoMovimientoBrazo = millis();
    
    if(DEBUG_MODE) {
      Serial.print(F("Paso ")); Serial.print(movimientoActual);
      Serial.print(F(" Pos:"));
    }
    
    for(byte i = 0; i < 6; i++) {
      uint16_t pos = pgm_read_word(&movimientos[movimientoActual][i]);
      moverServo(i, pos);
      if(DEBUG_MODE) { Serial.print(' '); Serial.print(pos); }
    }
    if(DEBUG_MODE) Serial.println();
    
    lcd2.setCursor(0, 1);
    lcd2.print(F("Brazo Paso "));
    lcd2.print(movimientoActual+1);
    lcd2.print('/');
    lcd2.print(sizeof(movimientos)/sizeof(movimientos[0]));
    lcd2.print("  ");
    
    if(++movimientoActual >= sizeof(movimientos)/sizeof(movimientos[0])) {
      movimientoActual = 0;
      brazoActivo = false;
      lcd2.setCursor(0, 1);
      lcd2.print(F("Brazo Inactivo  "));
      if(DEBUG_MODE) Serial.println(F("Secuencia completada"));
    }
  }
}

void activarBrazo() {
  if(!brazoActivo) {
    brazoActivo = true;
    movimientoActual = 0;
    tiempoMovimientoBrazo = millis();
    lcd2.setCursor(0, 1);
    lcd2.print(F("Brazo Activado  "));
    if(DEBUG_MODE) Serial.println(F("Brazo activado"));
    
    for(byte i = 0; i < 6; i++) {
      moverServo(i, pgm_read_word(&movimientos[0][i]));
    }
  }
}

// ... (resto de funciones optimizadas de manera similar)

void setup() {
  Serial.begin(9600);
  if(DEBUG_MODE) Serial.println(F("Iniciando sistema"));
  
  balanza.begin(DT, CLK);
  pwm.begin();
  pwm.setPWMFreq(30);
  
  // Configuración de pines y inicialización similar a la versión original
  // pero con strings en PROGMEM cuando sea posible
  
  // Mover a posición HOME
  for(byte i = 0; i < 6; i++) {
    moverServo(i, pgm_read_word(&movimientos[0][i]));
  }
  
  // Resto del setup...
}

void loop() {
  float peso = balanza.get_units(10);
  
  lcd.setCursor(1, 0);
  lcd.print(F("Peso: "));
  lcd.print(peso, 0);
  lcd.print(F(" g       "));
  
  if(DEBUG_MODE && millis() - lastDebugTime >= 5000) {
    lastDebugTime = millis();
    Serial.print(F("Peso: ")); Serial.print(peso, 0); Serial.println(F(" g"));
  }

  // Resto de la lógica principal optimizada...
  
  delay(10);
}