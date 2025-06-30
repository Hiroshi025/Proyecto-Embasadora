// Incluye las librerías necesarias
#include "HX711.h"
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <PCF8574.h>

// Define los pines de conexión
const byte DT = 3;
const byte CLK = 2;
const byte modo = 7;
const byte tara = 6;

// Configuración adicional para el proyecto
const byte led = 4;       // LED indicador
const byte electro1 = 8;  // Salida para la electroválvula de la tolva (HIGH = cerrada)
const byte electro2 = 9;  // Salida para el pistón (HIGH = extendido)
const byte electro3 = 5;  // Salida para el sellador (NUEVO)
const byte sensor1 = 10;  // Sensor de proximidad de frasco bajo tolva
const byte sensor2 = 11;  // Sensor de conteo de frascos
const byte led2 = 13;     // LED indicador de conteo
const byte sensor3 = 12;  // Sensor de zona de sellado

// Variables de conteo
int frascos = 0;
int cajas = 0;
bool estadoAnteriorSensor2 = LOW;

// Variables para control de tiempo
unsigned long tiempoPiston = 0;
unsigned long tiempoSellador = 0;
const unsigned long tiempoRetraccionPiston = 2000; // 2 segundos
const unsigned long tiempoEsperaSellado = 5000;    // 5 segundos antes de sellar (NUEVO)
const unsigned long tiempoDuracionSellado = 3000;  // 3 segundos de sellado (NUEVO)

// Variables para el estado del sistema
enum EstadoSistema {
  ESPERANDO_FRASCO,
  LLENANDO_FRASCO,
  EXPULSANDO_FRASCO,
  FRASCO_EN_BANDA,
  ESPERANDO_SELLADO,  // NUEVO estado
  SELLANDO_FRASCO
};
EstadoSistema estadoActual = ESPERANDO_FRASCO;

// Configuración de la balanza
int peso_conocido[4] = {500, 1000, 3000, 5000};
long escala;
const float limite_peso = 2000; // Límite de peso para activar electro2
const float peso_minimo = 50;   // Peso mínimo para considerar un frasco presente

// Crear los objetos para LCDs, balanza y PCF8574
LiquidCrystal_I2C lcd(0x27, 16, 2);    // LCD principal
LiquidCrystal_I2C lcd2(0x3F, 16, 2);   // LCD secundario
HX711 balanza;
PCF8574 pcf8574(0x20);

// Función de Anti-debounce
void anti_debounce(byte boton) {
  delay(100);
  while (digitalRead(boton));
  delay(100);
}

// Función de calibración y ajuste
void calibration() {
  int i = 0, cal = 1;
  long adc_lecture;

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
    }
  }
}

// Función para actualizar la información en la pantalla LCD secundaria
void actualizarLCD() {
    lcd2.clear();
    lcd2.setCursor(0, 0);
    lcd2.print("Frascos: ");
    lcd2.print(frascos);
    lcd2.setCursor(0, 1);
    lcd2.print("Cajas: ");
    lcd2.print(cajas);
}

// Función para controlar la banda transportadora
void controlBanda(bool activar) {
    pcf8574.write(0, activar ? LOW : HIGH); // LOW activa la banda
}

// Función para verificar condiciones de seguridad
bool condicionesSeguridad() {
    // Verificar si hay un frasco en la zona de sellado mientras se está llenando otro
    if ((estadoActual == LLENANDO_FRASCO || estadoActual == EXPULSANDO_FRASCO) && 
        digitalRead(sensor3) == HIGH) {
        lcd.setCursor(0, 1);
        lcd.print("ALERTA: Bloqueo ");
        return false;
    }
    
    // Verificar si el sellador se activó incorrectamente
    if (digitalRead(electro3) == HIGH && estadoActual != SELLANDO_FRASCO) {
        lcd.setCursor(0, 1);
        lcd.print("ALERTA: Sellador");
        digitalWrite(electro3, LOW);
        return false;
    }
    
    return true;
}

void setup() {
    // Inicialización de la comunicación serial para depuración
    Serial.begin(9600);
    
    // Configuración de la balanza
    balanza.begin(DT, CLK);
    pcf8574.begin();
    
    // Configuración de pines
    pinMode(electro1, OUTPUT);
    pinMode(electro2, OUTPUT);
    pinMode(electro3, OUTPUT); // NUEVO
    pinMode(sensor1, INPUT);
    pinMode(led, OUTPUT);
    pinMode(modo, INPUT);
    pinMode(tara, INPUT);
    pinMode(sensor2, INPUT);
    pinMode(led2, OUTPUT);
    pinMode(sensor3, INPUT);

    // Configurar PCF8574
    pcf8574.setButtonMask(0xF0); // Pines 4-7 como entradas
    
    // Inicializar actuadores en estado seguro
    digitalWrite(electro1, HIGH); // Tolva cerrada
    digitalWrite(electro2, LOW);  // Pistón retraído
    digitalWrite(electro3, LOW);  // Sellador desactivado (NUEVO)
    controlBanda(false);          // Banda detenida
    digitalWrite(led, HIGH);      // LED indicador encendido
    
    // Inicialización de LCDs
    lcd.init();
    lcd.backlight();
    lcd2.init();
    lcd2.backlight();
    
    // Cargar datos de EEPROM
    EEPROM.get(0, escala);
    EEPROM.get(2, cajas);
    actualizarLCD();
    
    // Calibración si se presionan ambos botones
    if (digitalRead(modo) && digitalRead(tara)) {
        calibration();
    }
    
    // Mensaje inicial
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
    
    // Mostrar peso actual en LCD principal
    lcd.setCursor(1, 0);
    lcd.print("Peso: ");
    lcd.print(peso, 0);
    lcd.print(" g       ");
    
    // Verificar condiciones de seguridad
    if (!condicionesSeguridad()) {
        // En caso de condición insegura, detener todo
        digitalWrite(electro1, HIGH);
        digitalWrite(electro2, LOW);
        digitalWrite(electro3, LOW); // NUEVO
        controlBanda(false);
        return;
    }
    
    // Máquina de estados principal
    switch (estadoActual) {
        case ESPERANDO_FRASCO:
            if (digitalRead(sensor1)) {
                // Frasco detectado bajo la tolva
                digitalWrite(electro1, LOW); // Abrir tolva
                estadoActual = LLENANDO_FRASCO;
                lcd.setCursor(1, 1);
                lcd.print("Llenando...    ");
            }
            break;
            
        case LLENANDO_FRASCO:
            if (peso >= limite_peso) {
                // Peso alcanzado, cerrar tolva
                digitalWrite(electro1, HIGH);
                estadoActual = EXPULSANDO_FRASCO;
                tiempoPiston = millis();
                digitalWrite(electro2, HIGH); // Extender pistón
                lcd.setCursor(1, 1);
                lcd.print("Expulsando...  ");
            }
            break;
            
        case EXPULSANDO_FRASCO:
            if (millis() - tiempoPiston >= tiempoRetraccionPiston) {
                digitalWrite(electro2, LOW); // Retraer pistón
                controlBanda(true); // Activar banda
                estadoActual = FRASCO_EN_BANDA;
                lcd.setCursor(1, 1);
                lcd.print("Transportando..");
            }
            break;
            
        case FRASCO_EN_BANDA:
            // Detección en el contador
            if (digitalRead(sensor2) && !estadoAnteriorSensor2) {
                frascos++;
                if (frascos >= 5) {
                    frascos = 0;
                    cajas++;
                    EEPROM.put(2, cajas);
                    digitalWrite(led2, HIGH);
                }
                actualizarLCD();
                digitalWrite(led2, HIGH);
                delay(50);
                digitalWrite(led2, LOW);
            }
            estadoAnteriorSensor2 = digitalRead(sensor2);
            
            // Detección en zona de sellado
            if (digitalRead(sensor3)) {
                controlBanda(false); // Detener banda
                tiempoSellador = millis();
                estadoActual = ESPERANDO_SELLADO; // NUEVO estado
                lcd.setCursor(1, 1);
                lcd.print("Preparando sellado");
            }
            
            // Verificar si el frasco ya no está bajo la tolva
            if (!digitalRead(sensor1)) {
                estadoActual = ESPERANDO_FRASCO;
                lcd.setCursor(1, 1);
                lcd.print("Esperando frasco");
            }
            break;
            
        case ESPERANDO_SELLADO: // NUEVO estado
            if (millis() - tiempoSellador >= tiempoEsperaSellado) {
                digitalWrite(electro3, HIGH); // Activar sellador
                tiempoSellador = millis();
                estadoActual = SELLANDO_FRASCO;
                lcd.setCursor(1, 1);
                lcd.print("Sellando...    ");
            }
            break;
            
        case SELLANDO_FRASCO:
            if (millis() - tiempoSellador >= tiempoDuracionSellado) {
                digitalWrite(electro3, LOW); // Desactivar sellador
                
                if (!digitalRead(sensor3)) {
                    // Frasco ha salido de la zona de sellado
                    controlBanda(true); // Reanudar banda
                    estadoActual = FRASCO_EN_BANDA;
                    lcd.setCursor(1, 1);
                    lcd.print("Transportando..");
                } else {
                    // Esperar a que el frasco salga de la zona de sellado
                    lcd.setCursor(1, 1);
                    lcd.print("Esperando salida");
                }
            }
            break;
    }
    
    // Botón de Tara
    if (digitalRead(tara)) {  
        anti_debounce(tara);
        balanza.tare(10);
    }
    
    // Pequeña pausa para evitar sobrecarga
    delay(10);
}