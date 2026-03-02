#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <arduino-sht.h>
#include <LittleFS.h>

extern "C" {
  #include "user_interface.h"
}

constexpr int interruptPin = 13;

volatile bool buttonPressed = false;

constexpr int SCREEN_WIDTH = 128;
constexpr int SCREEN_HEIGHT = 32;
constexpr int OLED_RESET = -1;
constexpr uint8_t OLED_ADDR = 0x3C;

const unsigned long SLEEP_MS = 20 * 60 * 1000; // 20 Minutes

const char* logFile = "/data.csv";

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

void flash_log(const float temperature, const float humidity)
{
  File file = LittleFS.open("/logs.txt", "a");
  if(file)
  {
    file.print("Temp: "); file.print(temperature);
    file.print(" Hum: "); file.println(humidity);
    file.close();
  }
}

void IRAM_ATTR buttonHandler()
{
  buttonPressed = true;
}

void setup() 
{
  Serial.begin(74880);
  Wire.begin();

  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), buttonHandler, FALLING);

  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  LittleFS.begin();
  sht.init();
  sht.setAccuracy(SHTSensor::SHT_ACCURACY_MEDIUM);
}

void loop() 
{
  if (buttonPressed) 
  {
    buttonPressed = false;
    Serial.println("Button was pressed!");

    if (sht.readSample()) 
    {
      const float temperature = sht.getTemperature();
      const float humidity = sht.getHumidity();

      uart_log(temperature, humidity);
      oled_log(temperature, humidity);
      flash_log(temperature, humidity);
    }
  }
}
