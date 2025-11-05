#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>

#define SDA_PIN 4   // GPIO4
#define SCL_PIN 5   // GPIO5
#define OLED_ADDR 0x3C
#define SHT_ADDR  0x40

Adafruit_SSD1306 display(128, 32, &Wire, -1);

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(100);

  // Инициализация OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("OLED not found"));
  } else {
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println(F("OLED OK"));
    display.display();
  }

  // Проверим SHT21
  Wire.beginTransmission(SHT_ADDR);
  if (Wire.endTransmission() == 0) {
    Serial.println(F("SHT21 found"));
  } else {
    Serial.println(F("SHT21 not found"));
  }
}

float readTemp() {
  Wire.beginTransmission(SHT_ADDR);
  Wire.write(0xF3); // measure temp, no hold
  Wire.endTransmission();
  delay(85);
  Wire.requestFrom(SHT_ADDR, (uint8_t)3);
  if (Wire.available() < 2) return NAN;
  uint16_t raw = (Wire.read() << 8) | Wire.read();
  raw &= ~0x0003;
  return -46.85 + 175.72 * (raw / 65536.0);
}

float readHumidity() {
  Wire.beginTransmission(SHT_ADDR);
  Wire.write(0xF5); // measure RH, no hold
  Wire.endTransmission();
  delay(30);
  Wire.requestFrom(SHT_ADDR, (uint8_t)3);
  if (Wire.available() < 2) return NAN;
  uint16_t raw = (Wire.read() << 8) | Wire.read();
  raw &= ~0x0003;
  return -6 + 125 * (raw / 65536.0);
}

void loop() {
  float t = readTemp();
  float h = readHumidity();

  Serial.print(F("T=")); Serial.print(t, 1);
  Serial.print(F("C  RH=")); Serial.print(h, 1);
  Serial.println(F("%"));

  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);
  if (!isnan(t)) { display.print(t, 1); display.print("C"); }
  else display.print("--.-C");
  display.setCursor(0, 18);
  if (!isnan(h)) { display.print(h, 1); display.print("%"); }
  else display.print("--.-%");
  display.display();

  delay(1000);
}
