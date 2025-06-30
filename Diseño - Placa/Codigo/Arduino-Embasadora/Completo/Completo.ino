/*
 * Sistema Automatizado de Envasado con Brazo Robótico
 * 
 * Flujo completo del proceso:
 * 1. El operador coloca un frasco vacío en la posición de llenado (sensor1)
 * 2. El sistema detecta el frasco y abre la válvula de llenado (electro1)
 * 3. La balanza monitorea el peso hasta alcanzar el objetivo (limite_peso)
 * 4. Al alcanzar el peso, se cierra la válvula y se activa el pistón (electro2)
 * 5. El pistón expulsa el frasco a la banda transportadora
 * 6. La banda transporta el frasco hasta la posición de sellado (sensor3)
 * 7. El sistema espera 5 segundos (tiempoEsperaSellado) antes de activar el sellador
 * 8. El sellador térmico se activa por 3 segundos (tiempoDuracionSellado)
 * 9. La banda transporta el frasco sellado fuera del sistema
 * 10. El brazo robótico se activa cuando se detecta frasco en la banda (sensor2)
 * 
 * El proceso se repite automáticamente mientras haya frascos vacíos
 * Cada 5 frascos se incrementa el contador de cajas
 * 
 * Estados posibles del sistema:
 * - ESPERANDO_FRASCO: Esperando detección de frasco vacío
 * - LLENANDO_FRASCO: Llenando el frasco con producto
 * - EXPULSANDO_FRASCO: Expulsando frasco lleno a la banda
 * - FRASCO_EN_BANDA: Frasco en movimiento en la banda transportadora
 * - ESPERANDO_SELLADO: Esperando tiempo previo al sellado
 * - SELLANDO_FRASCO: Activación del sellador térmico
 */

// *********************** BIBLIOTECAS ***********************
#include "HX711.h"                  // Para la celda de carga
#include <Wire.h>                   // Comunicación I2C
#include <LiquidCrystal_I2C.h>      // Control de LCDs I2C
#include <EEPROM.h>                 // Para guardar datos persistentes
//#include <Adafruit_PWMServoDriver.h> // Control del driver de servos

// *********************** DEFINICIÓN DE PINES ***********************
// Nota: Los sensores NPN son activos en LOW (0V cuando detectan)
const byte DT = 3; 
const byte CLK = 2; 
const byte modo = 7; 
const byte tara = 6; 
const byte banda = 8;
const byte electro2 = 10;
const byte electro3 = 5;
const byte electro1 = 9;
const byte sensor1 = 11;            // Sensor NPN de frasco en posición
const byte sensor2 = 12;            // Sensor NPN de frasco en banda
const byte led2 = 13;               // LED de indicación de conteo
const byte led = 4;                 // LED indicador de sistema listo
const byte sensor3 = A0;            // Sensor NPN de posición sellado (antes A0)
const byte ledAlerta = A1;          // Led de Alerta (antes A1)
const byte ledSellado = A2;         // Led de sellado (antes A2)

// *********************** CONFIGURACIÓN BRAZO ROBÓTICO ***********************
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40); // Dirección I2C del driver

// Rangos de movimiento para cada servo (en pulsos PWM)
//uint16_t servoPos[6] = {200, 200, 170, 550, 400, 333}; // Posición inicial (HOME)
//int servoMin[6] = {150, 150, 150, 150, 150, 150}; // Mínimos
//int servoMax[6] = {600, 600, 600, 600, 600, 600}; // Máximos

// *********************** CONSTANTES DE DEPURACIÓN ***********************
const bool DEBUG_MODE = true;               // Habilitar/deshabilitar salida de depuración
unsigned long lastDebugTime = 0;            // Último tiempo de log periódico

/*
// Secuencia completa de movimientos del brazo
const int movimientos[][6] = {
  // {S1, S2, S3, S4, S5, S6} - Posiciones para cada servo
  {150, 333, 290, 555, 456, 375}, // Posición HOME (reposo)
  {400, 350, 450, 500, 300, 375}, // Paso 1: Extender brazo
  {350, 400, 400, 450, 350, 375}, // Paso 2: Rotar hacia producto
  {300, 450, 350, 400, 400, 375}, // Paso 3: Agarrar producto
  {350, 400, 400, 450, 350, 375}, // Paso 4: Rotar a posición de entrega
  {432, 234, 599, 200, 300, 50}  // Paso 5: Retornar a HOME
};
*/

//byte movimientoActual = 0;           // Paso actual de la secuencia
//bool brazoActivo = false;           // Estado del brazo (activo/inactivo)
//unsigned long tiempoMovimientoBrazo = 0; // Temporizador para movimientos

// *********************** VARIABLES DEL SISTEMA ***********************
byte frascos = 0;                    // Contador de frascos procesados
byte cajas = 0;                      // Contador de cajas llenas (5 frascos/caja)
bool estadoAnteriorSensor2 = HIGH;  // Estado previo del sensor2 para detección de flanco

// Temporizadores para control de procesos
unsigned long tiempoPiston = 0; 
unsigned long tiempoSellador = 0;


// *********************** MÁQUINA DE ESTADOS ***********************
enum EstadoSistema {
  ESPERANDO_FRASCO,     // Esperando que coloquen frasco en posición
  LLENANDO_FRASCO,      // Llenando el frasco con producto
  EXPULSANDO_FRASCO,    // Expulsando frasco lleno
  FRASCO_EN_BANDA,      // Frasco en banda transportadora
  ESPERANDO_SELLADO,    // Esperando tiempo previo al sellado
  SELLANDO_FRASCO       // Sellando el frasco
};
EstadoSistema estadoActual = ESPERANDO_FRASCO; // Estado inicial

// *********************** CONFIGURACIÓN BALANZA ***********************
const uint16_t peso_conocido[5] = {500, 1000, 3000, 5000, 10000}; // Pesos para calibración (en gramos)
long escala;                                    // Factor de escala de la balanza
const float limite_peso = 2000, peso_minimo = 50;
// Peso objetivo por frasco (2000g)
// Peso mínimo para considerar válido

// *********************** DISPLAYS Y PERIFÉRICOS ***********************
LiquidCrystal_I2C lcd(0x27, 16, 2);   // LCD principal (dirección I2C 0x26)
LiquidCrystal_I2C lcd2(0x3F, 16, 2);  // LCD secundario (dirección I2C 0x27)
HX711 balanza;                         // Objeto para controlar la balanza

// *********************** FUNCIONES DEL BRAZO ROBÓTICO ***********************

/**
 * Mueve un servo a una posición específica con seguridad
 * @param servo Número de servo (0-5)
 * @param angulo Posición deseada (en pulsos PWM)
 */

 /*
void moverServo(byte servo, int angulo) {
  // Limitar el ángulo a los valores mínimos/máximos definidos
  angulo = constrain(angulo, servoMin[servo], servoMax[servo]);
  servoPos[servo] = angulo;
  pwm.setPWM(servo, 0, angulo);
  
  Serial.print("[DEBUG] Servo ");
  Serial.print(servo);
  Serial.print(" movido a: ");
  Serial.println(angulo);
}
*/

/**
 * Ejecuta el siguiente paso de la secuencia del brazo robótico
 */

 /*
void ejecutarSecuenciaBrazo() {
  if (brazoActivo && millis() - tiempoMovimientoBrazo >= 2000) {
    tiempoMovimientoBrazo = millis();
    char logMsg[50];
    sprintf(logMsg, "Ejecutando paso %d de secuencia", movimientoActual);
    
    Serial.print("[DEBUG] Ejecutando paso ");
    Serial.print(movimientoActual);
    Serial.print(" de la secuencia del brazo. Posiciones: ");
    
    // Mover todos los servos a la posición actual
    for (byte i = 0; i < 6; i++) {
      moverServo(i, movimientos[movimientoActual][i]);
      Serial.print(movimientos[movimientoActual][i]);
      Serial.print(" ");
    }
    Serial.println();
    
    // Mostrar estado en LCD secundario
    lcd2.setCursor(0, 1);
    lcd2.print("Brazo Paso ");
    lcd2.print(movimientoActual+1);
    lcd2.print("/");
    lcd2.print(sizeof(movimientos)/sizeof(movimientos[0]));
    lcd2.print("  ");
    
    // Avanzar a la siguiente posición
    movimientoActual++;
    
    // Verificar si la secuencia ha terminado
    if (movimientoActual >= sizeof(movimientos)/sizeof(movimientos[0])) {
      movimientoActual = 0;
      brazoActivo = false;
      lcd2.setCursor(0, 1);
      lcd2.print("Brazo Inactivo  ");
      if(DEBUG_MODE) Serial.println(F("[DEBUG] Secuencia del brazo completada"));
    }
  }
}
*/

/**
 * Activa el brazo robótico para comenzar su secuencia
 */

 /*
void activarBrazo() {
  if (!brazoActivo) {
    brazoActivo = true;
    movimientoActual = 0;
    tiempoMovimientoBrazo = millis();
    lcd2.setCursor(0, 1);
    lcd2.print("Brazo Activado  ");
    Serial.println("[DEBUG] Brazo robótico activado por sensor2");
    
    // Mover inmediatamente al primer paso de la secuencia
    for (byte i = 0; i < 6; i++) {
      moverServo(i, movimientos[0][i]);
    }
  }
}
*/

// *********************** FUNCIONES GENERALES ***********************

/**
 * Función anti-rebote para botones
 * @param boton Pin del botón a verificar
 */
void anti_debounce(byte boton) {
  delay(100);
  while (digitalRead(boton));
  delay(100);
}

/**
 * Procedimiento de calibración de la balanza
 */
void calibration() {
  int i = 0, cal = 1;
  long adc_lecture;

  Serial.println("[DEBUG] Iniciando proceso de calibración");
  
  lcd.setCursor(2, 0);
  lcd.print("Calibracion de");
  lcd.setCursor(4, 1);
  lcd.print("Balanza");
  delay(1500);
  
  // Preparar balanza para calibración
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

    // Cambiar peso de referencia con botón tara
    if (digitalRead(tara)) {
      anti_debounce(tara);
      i = (i > 2) ? 0 : i + 1;
      Serial.print("[DEBUG] Peso de calibración cambiado a: ");
      Serial.print(peso_conocido[i]);
      Serial.println(" g");
    }

    // Confirmar calibración con botón modo
    if (digitalRead(modo)) {
      lcd.clear();
      lcd.setCursor(1, 0);
      lcd.print("Ponga el Peso");
      lcd.setCursor(1, 1);
      lcd.print("y espere ...");
      delay(2000);

      // Obtener lectura promedio
      adc_lecture = balanza.get_value(100);
      escala = adc_lecture / peso_conocido[i];
      
      // Guardar en EEPROM
      EEPROM.put(0, escala);
      delay(100);
      cal = 0;
      lcd.clear();
      
      Serial.print("[DEBUG] Calibración completada. Escala: ");
      Serial.println(escala);
    }
  }
}

/**
 * Actualiza la información en el LCD secundario
 */
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

/**
 * Controla el estado de la banda transportadora
 * @param activar true para encender, false para apagar
 */
void controlBanda(bool activar) {
  digitalWrite(banda, activar ? HIGH : LOW);
  Serial.print("[DEBUG] Banda transportadora ");
  Serial.println(activar ? "activada" : "detenida");
}

/**
 * Verifica condiciones de seguridad del sistema
 * @return true si todo está en orden, false si hay condición insegura
 */
bool condicionesSeguridad() {
  // Bloqueo si hay obstrucción durante llenado o expulsión
  if ((estadoActual == LLENANDO_FRASCO || estadoActual == EXPULSANDO_FRASCO) && 
    digitalRead(sensor3) == LOW) {
    lcd.setCursor(0, 1);
    lcd.print("ALERTA: Obstrucción en sensor3");
    digitalWrite(ledAlerta, HIGH); // Encender LED de bloqueo
    Serial.println("[ALERTA] Bloqueo detectado: Frasco obstruyendo sensor3 durante llenado o expulsión.");
    return false;
  }
  
  // Detección de sellador activado en momento inadecuado
  if (digitalRead(electro3) == HIGH && estadoActual != SELLANDO_FRASCO) {
    lcd.setCursor(0, 1);
    lcd.print("ALERTA: Sellador activado fuera de estado");
    digitalWrite(electro3, LOW);
    digitalWrite(ledAlerta, HIGH); // Encender LED de bloqueo
    Serial.println("[ALERTA] Bloqueo detectado: Sellador activado en un estado no permitido.");
    return false;
  }
  
  digitalWrite(ledAlerta, LOW); // Apagar LED de bloqueo si no hay problemas
  return true;
}

// *********************** SETUP ***********************
void setup() {
  //Wire.begin();             // Inicia el bus I2C
  //Wire.setClock(100000);    // Configura la velocidad a 100 kHz (estándar)

  Serial.begin(9600);
  Serial.println("[DEBUG] Iniciando sistema de envasado");
  
  // Inicializar balanza
  balanza.begin(DT, CLK);
  
  // ************ Inicialización del Brazo Robótico ************
  //pwm.begin();
  //pwm.setPWMFreq(50);  // Frecuencia estándar para servos
  
  //Serial.println("[DEBUG] Inicializando servomotores...");
  
  // Mover todos los servos a posición inicial (HOME)
  /*for (byte i = 0; i < 6; i++) {
    moverServo(i, movimientos[0][i]);
  }*/
  
  // ************ Configuración de pines ************
  pinMode(electro1, OUTPUT);
  pinMode(electro2, OUTPUT);
  pinMode(electro3, OUTPUT);
  pinMode(sensor1, INPUT);    // Sensores NPN con pullup
  pinMode(led, OUTPUT);
  pinMode(modo, INPUT);
  pinMode(tara, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(led2, OUTPUT);
  pinMode(sensor3, INPUT);
  pinMode(banda, OUTPUT);
  pinMode(ledAlerta, OUTPUT);
  pinMode(ledSellado, OUTPUT);
  digitalWrite(ledAlerta, LOW);   // LED de bloqueo apagado inicialmente
  digitalWrite(ledSellado, LOW);   // LED de sellado apagado inicialmente

  // Estado inicial de actuadores
  digitalWrite(electro1, HIGH);  // Válvula de llenado cerrada
  digitalWrite(electro2, LOW);   // Pistón retraído
  digitalWrite(electro3, LOW);   // Sellador apagado
  controlBanda(false);           // Banda detenida
  digitalWrite(led, HIGH);       // LED de sistema listo
  
  // ************ Inicialización de displays ************
  lcd.init();
  lcd.backlight();
  lcd2.init();
  lcd2.backlight();
  
  // ************ Cargar configuración de EEPROM ************
  EEPROM.get(0, escala);  // Factor de escala de la balanza
  EEPROM.get(2, cajas);   // Contador de cajas persistente
  
  actualizarLCD();
  
  // ************ Modo de calibración (si se presionan ambos botones) ************
  if (digitalRead(modo) == LOW && digitalRead(tara) == LOW) {
    Serial.println("[DEBUG] Entrando en modo calibración");
    calibration();
  }
  
  // ************ Mensaje inicial ************
  lcd.setCursor(1, 0);
  lcd.print("Retire el Peso");
  lcd.setCursor(1, 1);
  lcd.print("y Espere");
  delay(2000);
  
  // Configurar balanza con valores cargados
  balanza.set_scale(escala);
  balanza.tare(20);  // Tara con 20 lecturas promedio
  
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print("Sistema Listo");
  delay(1000);
  lcd.clear();
  
  Serial.println("[DEBUG] Sistema inicializado completamente. Brazo en posición HOME");
}

// *********************** LOOP PRINCIPAL ***********************
void loop() {

  // Log periódico del estado del sistema
  if (millis() - lastDebugTime >= 5000) {
    lastDebugTime = millis();
  }

  // Leer peso actual con promedio de 10 lecturas
  float peso = balanza.get_units(10);
  
  // Mostrar peso en LCD principal
  lcd.setCursor(1, 0);
  lcd.print("Peso: ");
  lcd.print(peso, 0);
  lcd.print(" g       ");
  Serial.print("[DEBUG] Peso actual: ");
  Serial.print(peso, 0);
  Serial.println(" g");

  // Verificar condiciones de seguridad
  if (!condicionesSeguridad()) {
    // Estado de emergencia - detener todos los actuadores
    digitalWrite(electro1, HIGH);
    digitalWrite(electro2, LOW);
    digitalWrite(electro3, LOW);
    controlBanda(false);
    Serial.println("[DEBUG] Sistema detenido por condiciones inseguras.");
    return;
  }
  
  // ************ Control del Brazo Robótico ************
  // Detección de flanco descendente en sensor2 (NPN activo en LOW)
  if (digitalRead(sensor2) == LOW && estadoAnteriorSensor2 == HIGH) {
    frascos++;
    Serial.println(F("[DEBUG] Sensor2 activado - frasco detectado"));
    
    // Cada 5 frascos = 1 caja
    if (frascos >= 5) {
      frascos = 0;
      cajas++;
      EEPROM.put(2, cajas); // Guardar en EEPROM
      Serial.println(F("[DEBUG] Caja completada"));
    }
    actualizarLCD();
    
    // Activar el brazo si no está activo
    /*if (!brazoActivo) {
      activarBrazo();
    }*/
    
    // Feedback visual con LED
    digitalWrite(led2, HIGH);
    delay(50);
    digitalWrite(led2, LOW);
  }
  estadoAnteriorSensor2 = digitalRead(sensor2);
  
  // Ejecutar secuencia del brazo si está activo
  //ejecutarSecuenciaBrazo();
  
  // ************ MÁQUINA DE ESTADOS PRINCIPAL ************
  switch (estadoActual) {
    case ESPERANDO_FRASCO:
      Serial.println("[DEBUG] Estado: ESPERANDO_FRASCO");
      // Detección de frasco en posición (sensor1 NPN activo en LOW)
      if (digitalRead(sensor1) == LOW) {
        digitalWrite(electro1, LOW);  // Abrir válvula de llenado
        estadoActual = LLENANDO_FRASCO;
        lcd.setCursor(1, 1);
        lcd.print("Llenando...    ");
        Serial.println(F("[ESTADO] Cambio a LLENANDO_FRASCO"));
      }
      break;
      
    case LLENANDO_FRASCO:
      Serial.println("[DEBUG] Estado: LLENANDO_FRASCO");
      // Cuando se alcanza el peso objetivo
      if (peso >= limite_peso) {
        digitalWrite(electro1, HIGH); // Cerrar válvula
        estadoActual = EXPULSANDO_FRASCO;
        tiempoPiston = millis();
        digitalWrite(electro2, HIGH); // Activar pistón
        lcd.setCursor(1, 1);
        lcd.print("Expulsando...  ");
        Serial.println(F("[ESTADO] Cambio a EXPULSANDO_FRASCO"));
      }
      break;
      
    case EXPULSANDO_FRASCO:
      Serial.println("[DEBUG] Estado: EXPULSANDO_FRASCO");
      // Después de tiempoRetraccionPiston, retraer pistón
      if (millis() - tiempoPiston >= 2000) {
        digitalWrite(electro2, LOW);
        controlBanda(true);  // Activar banda
        estadoActual = FRASCO_EN_BANDA;
        lcd.setCursor(1, 1);
        lcd.print("Transportando..");
        Serial.println(F("[ESTADO] Cambio a FRASCO_EN_BANDA"));
      }
      break;
      
    case FRASCO_EN_BANDA:
      Serial.println("[DEBUG] Estado: FRASCO_EN_BANDA");
      // Cuando el frasco llega al sensor3 (posición de sellado)
      if (digitalRead(sensor3) == LOW) {
        controlBanda(false); // Detener banda
        tiempoSellador = millis();
        estadoActual = ESPERANDO_SELLADO;
        lcd.setCursor(1, 1);
        lcd.print("Preparando sellado");
        digitalWrite(ledSellado, HIGH); // Encender LED de sellado
        Serial.println(F("[ESTADO] Cambio a ESPERANDO_SELLADO"));
      } else {
        digitalWrite(ledSellado, LOW); // Apagar LED de sellado si no está en zona
      }
      
      // Si se detecta nuevo frasco, volver a estado inicial
      if (digitalRead(sensor1) == LOW) {
        estadoActual = ESPERANDO_FRASCO;
        lcd.setCursor(1, 1);
        lcd.print("Esperando frasco");
        Serial.println(F("[ESTADO] Cambio a ESPERANDO_FRASCO"));
      }
      break;
      
    case ESPERANDO_SELLADO:
      Serial.println("[DEBUG] Estado: ESPERANDO_SELLADO");
      // Espera previa al sellado
      if (millis() - tiempoSellador >= 5000) {
        digitalWrite(electro3, HIGH); // Activar sellador
        tiempoSellador = millis();
        estadoActual = SELLANDO_FRASCO;
        lcd.setCursor(1, 1);
        lcd.print("Sellando...    ");
        Serial.println(F("[ESTADO] Cambio a SELLANDO_FRASCO"));
      }
      break;
      
    case SELLANDO_FRASCO:
      Serial.println("[DEBUG] Estado: SELLANDO_FRASCO");
      // Finalizado el tiempo de sellado
      if (millis() - tiempoSellador >= 3000) {
        digitalWrite(electro3, LOW); // Desactivar sellador
        Serial.println("[DEBUG] Sellador desactivado");

        // Esperar 2 segundos antes de reactivar la banda
        if (millis() - tiempoSellador >= 5000) { // 3000ms de sellado + 2000ms de espera
            controlBanda(true);  // Reactivar banda
            estadoActual = FRASCO_EN_BANDA;
            lcd.setCursor(1, 1);
            lcd.print("Transportando..");
            Serial.println(F("[ESTADO] Cambio a FRASCO_EN_BANDA"));
        } else {
            lcd.setCursor(1, 1);
            lcd.print("Esperando salida");
            Serial.println(F("[DEBUG] Esperando 2 segundos para reactivar banda"));
        }
    }
    break;
  }
  
  // Función de tara si se presiona el botón
  if (digitalRead(tara) == LOW) {  
    anti_debounce(tara);
    balanza.tare(10);  // Tara con 10 lecturas promedio
    Serial.println(F("[DEBUG] Tara realizada"));
  }
  
  delay(10); // Pequeña pausa para estabilidad
}