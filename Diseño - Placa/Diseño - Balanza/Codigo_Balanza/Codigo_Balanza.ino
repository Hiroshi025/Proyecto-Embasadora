//Incluye las librerías necesarias
#include "HX711.h"
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

//Define los pines de conexión
byte DT = 3;
byte CLK = 2;
byte modo = 7;
byte tara = 6;

//Configuración adicional para el proyecto
byte led = 4;       // LED indicador
byte electro1 = 8;  // Salida para la electroválvula de la tolva
byte electro2 = 9;  // Salida para la electroválvula de los frascos
byte sensor1 = 10;  // Sensor de proximidad

//Definir pesos conocidos para la calibración
int peso_conocido[4] = {500, 1000, 3000, 5000};
long escala;
float limite_peso = 2000; // Límite de peso para activar electro2

//Crear el objeto LCD (dirección 0x27, 16 columnas x 2 filas)
LiquidCrystal_I2C lcd(0x27, 16, 2);

//Crear el objeto balanza
HX711 balanza;

//Función de Anti-debounce (Evitar rebote del pulsador)
void anti_debounce(byte boton) {
  delay(100);
  while (digitalRead(boton)); // Espera hasta que el botón se suelte
  delay(100);
}

//Función de calibración y ajuste
void calibration() {
  int i = 0, cal = 1;
  long adc_lecture;

  // Mensaje en el LCD
  lcd.setCursor(2, 0);
  lcd.print("Calibracion de");
  lcd.setCursor(4, 1);
  lcd.print("Balanza");
  delay(1500);
  
  balanza.read();
  balanza.set_scale(); // Escala por defecto es 1
  balanza.tare(20);    // Considerar el peso actual como Tara
  
  lcd.clear();

  // Proceso de ajuste y calibración
  while (cal == 1) {
    lcd.setCursor(1, 0);
    lcd.print("Peso Conocido:");
    lcd.setCursor(1, 1);
    lcd.print(peso_conocido[i]);
    lcd.print(" g        ");

    // Selección del peso conocido con el botón de tara
    if (digitalRead(tara)) {
      anti_debounce(tara);
      i = (i > 2) ? 0 : i + 1; // Cambio de peso conocido
    }

    // Confirmar el peso conocido con el botón de modo
    if (digitalRead(modo)) {
      lcd.clear();
      lcd.setCursor(1, 0);
      lcd.print("Ponga el Peso");
      lcd.setCursor(1, 1);
      lcd.print("y espere ...");
      delay(2000);

      // Leer el valor del HX711
      adc_lecture = balanza.get_value(100);

      // Calcular la escala
      escala = adc_lecture / peso_conocido[i];

      // Guardar la escala en la EEPROM
      EEPROM.put(0, escala);
      delay(100);
      cal = 0; // Salir del bucle de calibración
      lcd.clear();
    }
  }
}

void setup() {
  // Configuración de la balanza
  balanza.begin(DT, CLK);
  
  // Configuración de pines
  pinMode(led, OUTPUT);
  pinMode(modo, INPUT);
  pinMode(tara, INPUT);
  pinMode(sensor1, INPUT);
  pinMode(electro1, OUTPUT);
  pinMode(electro2, OUTPUT);
  
  // Inicialización del LCD y encendido del LED indicador
  digitalWrite(led, HIGH);
  lcd.init();
  lcd.backlight();

  // Leer la escala almacenada en la EEPROM
  EEPROM.get(0, escala);

  // Preguntar si se desea calibrar
  if (digitalRead(modo) && digitalRead(tara)) {
    calibration();
  }

  // Mensaje inicial en el LCD
  lcd.setCursor(1, 0);
  lcd.print("Retire el Peso");
  lcd.setCursor(1, 1);
  lcd.print("y Espere");
  delay(2000);
  
  balanza.set_scale(escala); // Establecer la escala calibrada
  balanza.tare(20);          // Considerar el peso actual como Tara

  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print("Listo....");
  delay(1000);
  lcd.clear();
}

void loop() {
  float peso;

  // Medir el peso en la balanza
  peso = balanza.get_units(10);

  // Mostrar el peso en el LCD
  lcd.setCursor(1, 0);
  lcd.print("Peso Actual: ");
  lcd.print(peso, 0);
  lcd.println(" g        ");
  delay(5);

  if (digitalRead(sensor1) == HIGH) { 
    digitalWrite(electro1, LOW); // Contraer la electroválvula de la tolva
    lcd.setCursor(1, 1);
    lcd.print("Tolva activada  ");
  } else {
    digitalWrite(electro1, HIGH); // Extender la electroválvula si no hay detección
    lcd.setCursor(1, 1);
    lcd.print("Tolva desactivada  ");
  }


  // Activación de electro2 cuando se alcanza el límite de peso
  if (peso >= limite_peso) {
    digitalWrite(electro2, HIGH); // Activar la electroválvula de los frascos
  } else {
    digitalWrite(electro2, LOW); // Apagar la electroválvula si no se alcanza el peso límite
  }

  // Botón de Tara para resetear el peso
  if (digitalRead(tara)) {  
    anti_debounce(tara);
    balanza.tare(10);  // Considerar el peso actual como Tara
  }
}
