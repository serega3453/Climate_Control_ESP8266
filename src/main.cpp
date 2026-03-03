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

const unsigned long SLEEP_MS = 30 * 60 * 1000; // 30 Minutes
unsigned long lastWakeTime = 0;

const unsigned long OLED_TIMEOUT_MS = 15 * 1000; // 15 seconds
unsigned long lastOledUpdateTime = 0;
bool oledOn = false;

struct sensorData 
{
  float temperature;
  float humidity;
  bool valid;
  unsigned long timestamp;
};

sensorData lastReading = {0, 0, false, 0};

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
SHTSensor sht(SHTSensor::SHT2X);

void uart_log()
{
  Serial.print("Temp: ");
  Serial.print(lastReading.temperature, 1);
  Serial.print(" C, Hum: ");
  Serial.print(lastReading.humidity, 1);
  Serial.println(" %");
}

void oled_log()
{
  display.ssd1306_command(SSD1306_DISPLAYON);
  oledOn = true;
  lastOledUpdateTime = millis();

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("Temp: ");
  display.print(lastReading.temperature, 1);
  display.println(" C");
  display.print("Hum : ");
  display.print(lastReading.humidity, 1);
  display.println(" %");
  display.display();
}

void flash_log()
{
  File file = LittleFS.open("/logs.txt", "a");
  if(file)
  {
    file.print("Temp: "); file.print(lastReading.temperature);
    file.print(" Hum: "); file.println(lastReading.humidity);
    file.close();
  }
}

bool getDataFromSensor()
{
  if (sht.readSample()) 
  {
    lastReading.temperature = sht.getTemperature();
    lastReading.humidity = sht.getHumidity();
    lastReading.valid = true;
    lastReading.timestamp = millis();
    return true;
  }
  else
  {
    lastReading.temperature = 0;
    lastReading.humidity = 0;
    lastReading.valid = false;
    lastReading.timestamp = millis();
    return false;
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

  wifi_set_opmode(NULL_MODE);
  wifi_fpm_set_sleep_type(MODEM_SLEEP_T);
  wifi_fpm_open();
  wifi_fpm_do_sleep(0xFFFFFFF);

  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), buttonHandler, FALLING);

  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  LittleFS.begin();
  sht.init();
  sht.setAccuracy(SHTSensor::SHT_ACCURACY_MEDIUM);

  if (getDataFromSensor()) {
    flash_log();
    uart_log();
  }
}

void loop() 
{
  if (oledOn && millis() - lastOledUpdateTime >= OLED_TIMEOUT_MS) 
  {
    display.ssd1306_command(SSD1306_DISPLAYOFF);
    oledOn = false;
  }

  if (buttonPressed) 
  {
    buttonPressed = false;
    Serial.println("Button was pressed");

    if (getDataFromSensor()) 
    {
      uart_log();
      flash_log();
      oled_log();
    }
  }

  if (millis() - lastWakeTime >= SLEEP_MS) 
  {
    lastWakeTime = millis();
    Serial.println("Waking up to log data");

    if (getDataFromSensor()) 
    {
      uart_log();
      flash_log();
    }
  }
}
