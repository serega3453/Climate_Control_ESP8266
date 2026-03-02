#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <arduino-sht.h>

constexpr int SCREEN_WIDTH = 128;
constexpr int SCREEN_HEIGHT = 32;
constexpr int OLED_RESET = -1;
constexpr uint8_t OLED_ADDR = 0x3C;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
SHTSensor sht(SHTSensor::SHT2X);

void uart_log(const float temperature, const float humidity)
{
  Serial.print("Temp: ");
  Serial.print(temperature, 1);
  Serial.print(" C, Hum: ");
  Serial.print(humidity, 1);
  Serial.println(" %");
}

void oled_log(const float temperature, const float humidity)
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("Temp: ");
  display.print(temperature, 1);
  display.println(" C");
  display.print("Hum : ");
  display.print(humidity, 1);
  display.println(" %");
  display.display();
}

void setup() 
{
  Serial.begin(74880);
  Wire.begin();

  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);

  sht.init();
  sht.setAccuracy(SHTSensor::SHT_ACCURACY_MEDIUM);
}

void loop() 
{
  if (sht.readSample()) 
  {
    const float temperature = sht.getTemperature();
    const float humidity = sht.getHumidity();

    uart_log(temperature, humidity);
    oled_log(temperature, humidity);
  }

  delay(1000);
}
