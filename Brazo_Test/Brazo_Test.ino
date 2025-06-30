#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver();

// Definición de servos (ajusta según tu configuración)
const uint8_t servos[6] = {0, 1, 2, 3, 4, 5}; // Canales 0-5 del PCA9685

// Límites de pulso para tus servos (debes calibrarlos)
const uint16_t SERVO_MIN[6] = {150, 150, 150, 150, 150, 150}; // Mínimos
const uint16_t SERVO_MAX[6] = {600, 600, 600, 600, 600, 600}; // Máximos
const uint16_t SERVO_INIT[6] = {375, 375, 375, 375, 375, 375}; // Posiciones iniciales

// Variables de posición actual
uint16_t currentPos[6];

void setup() {
  Serial.begin(9600);
  Serial.println("Inicializando control de brazo robotico con PCA9685");

  pca.begin();
  pca.setPWMFreq(60); // Frecuencia estándar para servos (60Hz)

  // Ir a posición inicial
  moveToInitialPosition();
  delay(2000);
}

void loop() {
  if (Serial.available()) {
    char command = Serial.read();
    
    switch(command) {
      case '1': movement1(); break;
      case '2': movement2(); break;
      case '3': movement3(); break;
      case '4': movement4(); break;
      case '5': movement5(); break;
      case '6': movement6(); break;
      case 'h': moveToInitialPosition(); break;
      default:
        printMenu();
    }
  }
}

void printMenu() {
  Serial.println("\n=== Menu de Control ===");
  Serial.println("1 - Movimiento 1: Rotación de base");
  Serial.println("2 - Movimiento 2: Hombro arriba/abajo");
  Serial.println("3 - Movimiento 3: Codo flexion/extensión");
  Serial.println("4 - Movimiento 4: Muñeca vertical");
  Serial.println("5 - Movimiento 5: Pinza abrir/cerrar");
  Serial.println("6 - Movimiento 6: Secuencia completa");
  Serial.println("h - Home (posición inicial)");
  Serial.println("Seleccione una opción:");
}

void moveToInitialPosition() {
  Serial.println("Moviendo a posición inicial...");
  for (int i = 0; i < 6; i++) {
    smoothMove(servos[i], SERVO_INIT[i]);
    currentPos[i] = SERVO_INIT[i];
  }
}

void smoothMove(uint8_t servoNum, uint16_t targetPos) {
  // Limitar el rango de movimiento
  targetPos = constrain(targetPos, SERVO_MIN[servoNum], SERVO_MAX[servoNum]);
  
  // Movimiento suave con interpolación
  int step = (targetPos > currentPos[servoNum]) ? 1 : -1;
  
  while (currentPos[servoNum] != targetPos) {
    currentPos[servoNum] += step;
    pca.setPWM(servos[servoNum], 0, currentPos[servoNum]);
    
    // Ajustar esta velocidad según necesites
    delay(20);
    
    // Limitar en caso de sobrepaso
    if ((step > 0 && currentPos[servoNum] > targetPos) ||
        (step < 0 && currentPos[servoNum] < targetPos)) {
      currentPos[servoNum] = targetPos;
    }
  }
}

// Movimientos predefinidos para pruebas
void movement1() { // Rotación de base
  Serial.println("Movimiento 1: Rotación de base");
  smoothMove(0, 150); // Giro izquierdo
  delay(500);
  smoothMove(0, 600); // Giro derecho
  delay(500);
  smoothMove(0, SERVO_INIT[0]); // Volver al centro
}

void movement2() { // Hombro arriba/abajo
  Serial.println("Movimiento 2: Hombro");
  smoothMove(1, 150); // Hombro arriba
  delay(500);
  smoothMove(1, 600); // Hombro abajo
  delay(500);
  smoothMove(1, SERVO_INIT[1]); // Posición media
}

void movement3() { // Codo flexion/extensión
  Serial.println("Movimiento 3: Codo");
  smoothMove(2, 150); // Codo flexionado
  delay(500);
  smoothMove(2, 600); // Codo extendido
  delay(500);
  smoothMove(2, SERVO_INIT[2]); // Posición media
}

void movement4() { // Muñeca vertical
  Serial.println("Movimiento 4: Muñeca");
  smoothMove(3, 150); // Muñeca arriba
  delay(500);
  smoothMove(3, 600); // Muñeca abajo
  delay(500);
  smoothMove(3, SERVO_INIT[3]); // Posición media
}

void movement5() { // Pinza abrir/cerrar
  Serial.println("Movimiento 5: Pinza");
  smoothMove(4, 150); // Pinza cerrada
  delay(500);
  smoothMove(4, 600); // Pinza abierta
  delay(500);
  smoothMove(4, SERVO_INIT[4]); // Semi-abierta
}

void movement6() { // Secuencia completa
  Serial.println("Movimiento 6: Secuencia completa");
  movement1();
  delay(300);
  movement2();
  delay(300);
  movement3();
  delay(300);
  movement4();
  delay(300);
  movement5();
}