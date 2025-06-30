#include <Wire.h>
#include <LiquidCrystal_I2C.h>

byte lcdAddress = 0x27;  // Dirección por defecto
LiquidCrystal_I2C lcd(lcdAddress, 16, 2); 

void scanI2C() {
  Serial.println("🔍 Escaneando dispositivos I2C...");
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.print("✅ Dispositivo encontrado en 0x");
      Serial.println(address, HEX);
      lcdAddress = address;
    }
  }
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  scanI2C();  // Escanea y detecta la dirección I2C

  Serial.print("Usando dirección: 0x");
  Serial.println(lcdAddress, HEX);

  lcd = LiquidCrystal_I2C(lcdAddress, 16, 2);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("LCD funcionando!");
}

void loop() {
  lcd.setCursor(0, 1);
  lcd.print(millis() / 1000);
  delay(1000);
}
