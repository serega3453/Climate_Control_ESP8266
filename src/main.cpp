#include <Arduino.h>
#include <Wire.h>

// ===== OLED (Adafruit SSD1306) =====
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------- ПИНЫ / АДРЕСА / НАСТРОЙКИ ----------
#define SDA_PIN           4      // GPIO4
#define SCL_PIN           5      // GPIO5
#define OLED_ADDR         0x3C
#define SHT_ADDR          0x40
#define OLED_WIDTH        128
#define OLED_HEIGHT       32
#define OLED_RESET_PIN    -1     // -1: аппаратный reset не используем

// Интервалы
#define OLED_SHOW_MS      5000   // сколько держать экран включенным после измерения
#define SLEEP_SECONDS     10    // интервал глубокого сна, секунд

// АЦП / батарея
// Если чистый ESP-12F (ADC max ~1.0 В) + внешний делитель → задайте коэффициент делителя.
// Пример: R1=220k на плюс батареи, R2=100k на GND, точка к АЦП. Коэффициент K = (R1+R2)/R2 = 3.2
// Тогда 4.2 В / 3.2 ≈ 1.31 В (в норме нужно уложиться до референса; скорректируйте резисторы под свою плату)
#define ADC_REF_VOLT      1.000  // опорное напряжение АЦП ESP8266 (чистый модуль) ~1.0 В
#define ADC_DIV_RATIO     3.20   // коэффициент делителя (напряжение батареи / напряжение на пине A0)
// Если у вас NodeMCU/WeMos с встроенным делителем до 3.3В: поставьте ADC_REF_VOLT=3.30 и ADC_DIV_RATIO=1.0

// Отладка по Serial (можно отключить)
#define DEBUG_SERIAL
#ifdef DEBUG_SERIAL
  #define DBG(...) Serial.printf(__VA_ARGS__)
#else
  #define DBG(...)
#endif

Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET_PIN);

// ====== Команды SHT21/HTU21 ======
static const uint8_t CMD_SOFT_RESET   = 0xFE;
static const uint8_t CMD_TEMP_NOHOLD  = 0xF3;
static const uint8_t CMD_RH_NOHOLD    = 0xF5;

// CRC для SHT21: полином x^8 + x^5 + x^4 + 1 → 0x131 (9 бит), реализуем побитово
uint8_t sht_crc8(const uint8_t *data, int len) {
  uint8_t crc = 0x00;
  for (int i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) {
      if (crc & 0x80) crc = (crc << 1) ^ 0x31;  // 0x31 = 0b0011_0001 (младшие 8 бит от 0x131)
      else            crc <<= 1;
    }
  }
  return crc;
}

bool sht_softReset() {
  Wire.beginTransmission(SHT_ADDR);
  Wire.write(CMD_SOFT_RESET);
  if (Wire.endTransmission() != 0) return false;
  delay(15); // даташит: ≥15 мс
  return true;
}

// чтение 2 байтов данных + CRC из датчика; cmd = CMD_TEMP_NOHOLD / CMD_RH_NOHOLD
bool sht_readRaw(uint8_t cmd, uint16_t &rawOut) {
  // Запрос измерения (без удержания шины)
  Wire.beginTransmission(SHT_ADDR);
  Wire.write(cmd);
  if (Wire.endTransmission() != 0) return false;

  // Подождать завершение конверсии
  // Температура max ~85 мс, влажность max ~29 мс. Возьмём с запасом.
  delay(cmd == CMD_TEMP_NOHOLD ? 100 : 50);

  // Прочитать 3 байта: MSB, LSB, CRC
  Wire.requestFrom((int)SHT_ADDR, 3);
  if (Wire.available() < 3) return false;

  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  uint8_t crc = Wire.read();

  // Проверка CRC
  uint8_t buf[2] = {msb, lsb};
  if (sht_crc8(buf, 2) != crc) return false;

  uint16_t raw = ((uint16_t)msb << 8) | lsb;
  raw &= 0xFFFC; // очистить статусные биты [1:0]
  rawOut = raw;
  return true;
}

bool sht_read(float &tC, float &rh) {
  uint16_t rt = 0, rrh = 0;

  // Иногда полезно сделать soft reset при каждом старте
  if (!sht_softReset()) return false;

  if (!sht_readRaw(CMD_TEMP_NOHOLD, rt)) return false;
  if (!sht_readRaw(CMD_RH_NOHOLD, rrh))  return false;

  // Преобразование по даташиту
  tC  = -46.85f + 175.72f * ( (float)rt  / 65536.0f );
  rh  =  -6.00f + 125.00f * ( (float)rrh / 65536.0f );

  // Ограничить RH в [0..100]
  if (rh < 0) rh = 0; if (rh > 100) rh = 100;
  return true;
}

float readBatteryVoltage() {
  // Несколько выборок для усреднения
  const int N = 8;
  uint32_t acc = 0;
  for (int i = 0; i < N; i++) {
    acc += analogRead(A0);
    delay(2);
  }
  float adc = (float)acc / (float)N;
  // Перевод из кода АЦП в вольты на пине A0:
  float vA0 = (adc / 1023.0f) * ADC_REF_VOLT;
  // Возврат к напряжению батареи
  return vA0 * ADC_DIV_RATIO;
}

void oledShowData(float tC, float rh, float vbat) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(2);
  display.setCursor(0, 0);
  display.printf("T:%2.1fC", tC);

  display.setTextSize(2);
  display.setCursor(0, 16);
  display.printf("H:%2.0f%%", rh);

  // Короткая строка с батареей (мелким шрифтом в правом углу)
  display.setTextSize(1);
  char buf[16];
  snprintf(buf, sizeof(buf), "Bat:%.2fV", vbat);
  int16_t x1, y1; uint16_t w, h;
  display.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);
  display.setCursor(OLED_WIDTH - w - 2, 0);
  display.print(buf);

  display.display();
}

void oledOn()  { display.ssd1306_command(SSD1306_DISPLAYON); }
void oledOff() { display.ssd1306_command(SSD1306_DISPLAYOFF); }

void initPeripherals() {
#ifdef DEBUG_SERIAL
  Serial.begin(74880);
  delay(50);
  DBG("\nBoot...\n");
#endif

  // I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  // Снизим частоту для длинного провода датчика (при необходимости):
  // Wire.setClock(100000); // 100 кГц — безопасно для 1–1.5 м

  // OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    // Если дисплей не нашёлся — просто продолжим без него
    DBG("OLED init failed\n");
  } else {
    oledOn();
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Clima v1.0");
    display.println("Init...");
    display.display();
  }

  // АЦП
  analogRead(A0); // "пустое" чтение для стабилизации
}

void goToDeepSleep() {
  DBG("Sleep for %d s\n", SLEEP_SECONDS);
  oledOff();
  ESP.deepSleep((uint64_t)SLEEP_SECONDS * 1000000ULL);
}

void setup() {
  initPeripherals();

  // Измерение
  float tC = NAN, rh = NAN;
  bool ok = sht_read(tC, rh);
  if (!ok) {
    DBG("SHT read failed\n");
    // Попробуем ещё один раз через короткую паузу
    delay(50);
    ok = sht_read(tC, rh);
  }

  float vbat = readBatteryVoltage();

  DBG("T=%.2f C, RH=%.1f %%, Bat=%.2f V\n", tC, rh, vbat);

  // Вывод на экран (если доступен)
  oledShowData(isnan(tC) ? -99.9f : tC,
               isnan(rh) ?  -1.0f  : rh,
               vbat);

  // Подержать экран включенным
  uint32_t t0 = millis();
  while (millis() - t0 < OLED_SHOW_MS) {
    delay(10);
  }

  goToDeepSleep();
}

void loop() {
  // Не используется: работа в one-shot перед сном
}
