/*
 * Sistema Automatizado de Envasado con Brazo Robótico - Versión Corregida
 */

#include "HX711.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <Adafruit_PWMServoDriver.h>

// Definición de pines
const byte DT = 3;
const byte CLK = 2;
const byte modo = 7;
const byte tara = 6;
const byte banda = 8;
const byte electro2 = 10;
const byte electro1 = 9;
const byte sensor1 = 11;
const byte sensor2 = 12;
const byte led2 = 13;
const byte led = 4;
const byte ledAlerta = A1;

// Configuración del brazo robótico
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
uint16_t servoPos[6] = {200, 200, 170, 550, 400, 333};
int servoMin[6] = {150, 150, 150, 150, 150, 150};
int servoMax[6] = {600, 600, 600, 600, 600, 600};

const int movimientos[][6] = {
  {150, 333, 290, 555, 456, 375},
  {400, 350, 450, 500, 300, 375},
  {350, 400, 400, 450, 350, 375},
  {300, 450, 350, 400, 400, 375},
  {350, 400, 400, 450, 350, 375},
  {432, 234, 599, 200, 300, 50}
};

// Variables del sistema
byte movimientoActual = 0;
bool brazoActivo = false;
unsigned long tiempoMovimientoBrazo = 0;
byte frascos = 0;
byte cajas = 0;
bool estadoAnteriorSensor2 = HIGH;
unsigned long tiempoPiston = 0;
unsigned long lastDebugTime = 0;

// Estados del sistema
enum EstadoSistema {
  ESPERANDO_FRASCO,
  LLENANDO_FRASCO,
  EXPULSANDO_FRASCO,
  FRASCO_EN_BANDA
};
EstadoSistema estadoActual = ESPERANDO_FRASCO;

// Configuración balanza
const uint16_t peso_conocido[5] = {500, 1000, 3000, 5000, 10000};
long escala;
const float limite_peso = 2000, peso_minimo = 50;

// Displays
LiquidCrystal_I2C lcd(0x27, 16, 2);
LiquidCrystal_I2C lcd2(0x3F, 16, 2);
HX711 balanza;

// Declaración de funciones (prototipos)
void mostrarEstados();
void moverServo(byte servo, int angulo);
void ejecutarSecuenciaBrazo();
void activarBrazo();
void anti_debounce(byte boton);
void calibration();
void actualizarLCD();
void controlBanda(bool activar);
bool condicionesSeguridad();

// Implementación de funciones
void mostrarEstados() {
  Serial.println("\n===== ESTADO ACTUAL DEL SISTEMA =====");
  Serial.print("Estado del sistema: ");
  switch(estadoActual) {
    case ESPERANDO_FRASCO: Serial.println("ESPERANDO_FRASCO"); break;
    case LLENANDO_FRASCO: Serial.println("LLENANDO_FRASCO"); break;
    case EXPULSANDO_FRASCO: Serial.println("EXPULSANDO_FRASCO"); break;
    case FRASCO_EN_BANDA: Serial.println("FRASCO_EN_BANDA"); break;
  }
  
  Serial.println("\nSENSORES:");
  Serial.print("  sensor1 (Frasco vacio): ");
  Serial.println(digitalRead(sensor1) == LOW ? "DETECTADO (LOW)" : "NO detectado (HIGH)");
  Serial.print("  sensor2 (Frasco banda): ");
  Serial.println(digitalRead(sensor2) == LOW ? "DETECTADO (LOW)" : "NO detectado (HIGH)");
  
  Serial.println("\nACTUADORES:");
  Serial.print("  electro1 (Valvula): ");
  Serial.println(digitalRead(electro1) == LOW ? "ABIERTA (LOW)" : "CERRADA (HIGH)");
  Serial.print("  electro2 (Piston): ");
  Serial.println(digitalRead(electro2) == HIGH ? "ACTIVADO (HIGH)" : "RETRAIDO (LOW)");
  Serial.print("  Banda transportadora: ");
  Serial.println(digitalRead(banda) == HIGH ? "ENCENDIDA (HIGH)" : "APAGADA (LOW)");
  
  Serial.println("\nCONTADORES:");
  Serial.print("  Frascos: "); Serial.println(frascos);
  Serial.print("  Cajas: "); Serial.println(cajas);
  
  Serial.print("Peso actual: ");
  Serial.print(balanza.get_units(1), 0);
  Serial.println(" g");
  
  Serial.println("===============================\n");
}

void moverServo(byte servo, int angulo) {
  angulo = constrain(angulo, servoMin[servo], servoMax[servo]);
  servoPos[servo] = angulo;
  pwm.setPWM(servo, 0, angulo);
}

void ejecutarSecuenciaBrazo() {
  if (brazoActivo && millis() - tiempoMovimientoBrazo >= 2000) {
    tiempoMovimientoBrazo = millis();
    
    for (byte i = 0; i < 6; i++) {
      moverServo(i, movimientos[movimientoActual][i]);
    }
    
    movimientoActual++;
    
    if (movimientoActual >= sizeof(movimientos)/sizeof(movimientos[0])) {
      movimientoActual = 0;
      brazoActivo = false;
    }
  }
}

void activarBrazo() {
  if (!brazoActivo) {
    brazoActivo = true;
    movimientoActual = 0;
    tiempoMovimientoBrazo = millis();
    
    for (byte i = 0; i < 6; i++) {
      moverServo(i, movimientos[0][i]);
    }
  }
}

void anti_debounce(byte boton) {
  delay(100);
  while (digitalRead(boton));
  delay(100);
}

void calibration() {
  int i = 0, cal = 1;
  long adc_lecture;

  Serial.println("[DEBUG] Iniciando proceso de calibración");
  
  lcd.setCursor(2, 0);
  lcd.print("Calibracion de");
  lcd.setCursor(4, 1);
  lcd.print("Balanza");
  delay(1500);
  
  balanza.read();
  balanza.set_scale();
  balanza.tare(20);
  
  lcd.clear();

  while (cal == 1) {
    lcd.setCursor(1, 0);
    lcd.print("Peso Conocido:");
    lcd.setCursor(1, 1);
    lcd.print(peso_conocido[i]);
    lcd.print(" g        ");

    if (digitalRead(tara)) {
      anti_debounce(tara);
      i = (i > 2) ? 0 : i + 1;
      Serial.print("[DEBUG] Peso de calibración cambiado a: ");
      Serial.print(peso_conocido[i]);
      Serial.println(" g");
    }

    if (digitalRead(modo)) {
      lcd.clear();
      lcd.setCursor(1, 0);
      lcd.print("Ponga el Peso");
      lcd.setCursor(1, 1);
      lcd.print("y espere ...");
      delay(2000);

      adc_lecture = balanza.get_value(100);
      escala = adc_lecture / peso_conocido[i];
      EEPROM.put(0, escala);
      delay(100);
      cal = 0;
      lcd.clear();
      Serial.print("[DEBUG] Calibración completada. Escala: ");
      Serial.println(escala);
    }
  }
}

void actualizarLCD() {
  lcd2.clear();
  lcd2.setCursor(0, 0);
  lcd2.print("Frascos: ");
  lcd2.print(frascos);
  lcd2.setCursor(0, 1);
  lcd2.print("Cajas: ");
  lcd2.print(cajas);
  Serial.print("[DEBUG] Contadores - Frascos: ");
  Serial.print(frascos);
  Serial.print(", Cajas: ");
  Serial.println(cajas);
}

void controlBanda(bool activar) {
  digitalWrite(banda, activar ? HIGH : LOW);
  Serial.print("[DEBUG] Banda transportadora ");
  Serial.println(activar ? "activada" : "detenida");
}

bool condicionesSeguridad() {
  if ((estadoActual == LLENANDO_FRASCO || estadoActual == EXPULSANDO_FRASCO) && 
    digitalRead(sensor2) == LOW) {
    lcd.setCursor(0, 1);
    lcd.print("ALERTA: Obstrucción");
    digitalWrite(ledAlerta, HIGH);
    Serial.println("[ALERTA] Bloqueo detectado");
    return false;
  }
  
  digitalWrite(ledAlerta, LOW);
  return true;
}

void setup() {
  Wire.begin();
  Wire.setClock(100000);
  Serial.begin(9600);
  
  // Inicializar brazo robótico
  pwm.begin();
  pwm.setPWMFreq(50);
  Serial.println("[DEBUG] Inicializando servomotores...");
  for (byte i = 0; i < 6; i++) {
    moverServo(i, movimientos[0][i]);
  }

  // Configurar pines
  pinMode(electro1, OUTPUT);
  pinMode(electro2, OUTPUT);
  pinMode(sensor1, INPUT);
  pinMode(led, OUTPUT);
  pinMode(modo, INPUT);
  pinMode(tara, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(led2, OUTPUT);
  pinMode(banda, OUTPUT);
  pinMode(ledAlerta, OUTPUT);
  
  digitalWrite(electro1, HIGH);
  digitalWrite(electro2, LOW);
  controlBanda(false);
  digitalWrite(led, HIGH);
  digitalWrite(ledAlerta, LOW);

  // Inicializar displays
  lcd.init();
  lcd.backlight();
  lcd2.init();
  lcd2.backlight();

  // Cargar configuración
  EEPROM.get(0, escala);
  EEPROM.get(2, cajas);
  actualizarLCD();

  if (digitalRead(modo) == LOW && digitalRead(tara) == LOW) {
    calibration();
  }

  lcd.setCursor(1, 0);
  lcd.print("Retire el Peso");
  lcd.setCursor(1, 1);
  lcd.print("y Espere");
  delay(2000);

  balanza.set_scale(escala);
  balanza.tare(20);

  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print("Sistema Listo");
  delay(1000);
  lcd.clear();
}

void loop() {
  float peso = balanza.get_units(10);
  
  // Mostrar información de debug cada 2 segundos
  if (millis() - lastDebugTime >= 2000) {
    mostrarEstados();
    lastDebugTime = millis();
  }

  // Control del brazo robótico
  if (digitalRead(sensor2) == LOW && estadoAnteriorSensor2 == HIGH) {
    frascos++;
    if (frascos >= 5) {
      frascos = 0;
      cajas++;
      EEPROM.put(2, cajas);
    }
    actualizarLCD();
    if (!brazoActivo) {
      activarBrazo();
    }
    digitalWrite(led2, HIGH);
    delay(50);
    digitalWrite(led2, LOW);
  }
  estadoAnteriorSensor2 = digitalRead(sensor2);
  ejecutarSecuenciaBrazo();

  // Máquina de estados principal
  switch (estadoActual) {
    case ESPERANDO_FRASCO:
      if (digitalRead(sensor1) == LOW) {
        digitalWrite(electro1, LOW);
        estadoActual = LLENANDO_FRASCO;
      }
      break;
      
    case LLENANDO_FRASCO:
      if (peso >= limite_peso) {
        digitalWrite(electro1, HIGH);
        estadoActual = EXPULSANDO_FRASCO;
        tiempoPiston = millis();
        digitalWrite(electro2, HIGH);
      }
      break;
      
    case EXPULSANDO_FRASCO:
      if (millis() - tiempoPiston >= 2000) {
        digitalWrite(electro2, LOW);
        controlBanda(true);
        estadoActual = FRASCO_EN_BANDA;
      }
      break;
      
    case FRASCO_EN_BANDA:
      if (digitalRead(sensor1) == LOW) {
        estadoActual = ESPERANDO_FRASCO;
      }
      break;
  }

  if (digitalRead(tara) == LOW) {
    anti_debounce(tara);
    balanza.tare(10);
  }
  
  delay(10);
}