#include <Arduino.h>
#include <SPI.h>
#include <U8g2lib.h>
#include <RadioLib.h>
#include <ds18b20.h>
#include <LoraHomeProtocol.h>
// TODO: Добавить отображение заряда батареи(либо как значок, либо процентами)
// TODO: Добавить повторную отправку пакета, если не пришёл ответ
// TODO: Улучшить глубокий сон
// TODO: отладить логику отправки и приёма
// TODO: Сделать README.md
constexpr uint8_t BUTTON_PIN = 0;

// !!! CHOOSE MODE BEFORE COMPILING !!!
 #define MASTER_MODE // - request the telemetry
// #define SLAVE_MODE // - response the telemetry

// PROTOCOL_LOGIC
constexpr uint8_t MASTER_ID = 0x01;
constexpr uint8_t SLAVE_ID = 0x02;
constexpr uint16_t TIMEOUT_TIME = 10000;

// OLED
constexpr uint8_t OLED_SCL = 18;
constexpr uint8_t OLED_SDA = 17;
constexpr uint8_t OLED_RST = 21;
constexpr uint8_t VEXT_PIN = 36;

// LORA
constexpr uint8_t LORA_NSS   = 8;
constexpr uint8_t LORA_DIO1  = 14;
constexpr uint8_t LORA_NRST  = 12;
constexpr uint8_t LORA_BUSY = 13;

// GLOBAL VARIABLES
String lastText = "";
volatile bool interruptFlag = false;
unsigned long lastTime = 0;

OneWire oneWire(4);
DallasTemperature sensor(&oneWire);

U8G2_SSD1306_128X64_NONAME_1_SW_I2C u8g2(U8G2_R0, /* clock=*/OLED_SCL, /* data=*/OLED_SDA, /* reset=*/OLED_RST);
// U8G2_SSD1306_128X64_NONAME_1_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);   // All Boards without Reset of the Display
// U8G2_SSD1306_128X64_NONAME_1_SW_I2C u8g2(U8G2_R0, /* clock=*/ 16, /* data=*/ 17, /* reset=*/ U8X8_PIN_NONE);   // ESP32 Thing, pure SW emulated I2C
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY);

void deepSleepOn(SX1262 &radio) {
  esp_sleep_enable_ext0_wakeup(gpio_num_t(BUTTON_PIN), 0);
  radio.sleep();
  digitalWrite(VEXT_PIN, LOW);
  esp_deep_sleep_start();
}

bool buttonHold(unsigned long& lastPressedTime, bool& button_pressed, uint16_t hold_time_ms, uint16_t timeout_hold_ms, uint8_t button_pin)
{

  if (digitalRead(button_pin) == LOW && !button_pressed)
  {
    button_pressed = true;

    lastPressedTime = millis();
  }

  if (millis() - lastPressedTime >= hold_time_ms && millis() - lastPressedTime < timeout_hold_ms && button_pressed)
  {
    if (digitalRead(button_pin) == HIGH)
    {
      lastPressedTime = 0;
      button_pressed = false;
      return true;
    }
  } else if (button_pressed && digitalRead(button_pin) == HIGH)
    {
      button_pressed = false;
      lastPressedTime = 0;
      return false;
    }

    return false;

}

bool buttonFastPressed(uint16_t hold_time_ms, uint16_t timeout_hold_ms, uint8_t button_pin)
{
  static unsigned long lastPressed = 0;
  static bool button_pressed = false;
  return buttonHold(lastPressed, button_pressed, hold_time_ms, timeout_hold_ms, button_pin);
}

void setFlag()
{
  interruptFlag = true;
}
void printText(const String &s, String &lastText)
{
  if (s == lastText)
  {
    return;
  }
  lastText = s;
  u8g2.clearDisplay();
  u8g2.firstPage();
  do
  {
    u8g2.setFont(u8g2_font_6x13_t_cyrillic);
    u8g2.setCursor(0, 20);
    u8g2.print(s);
  } while (u8g2.nextPage());
}

void printTwoLines(const String &line1, const String &line2, String &lastText)
{
  String currentText = line1 + "|" + line2;
  if (currentText == lastText)
  {
    return;
  }
  lastText = currentText;

  u8g2.clearDisplay();
  u8g2.firstPage();
  do
  {
    u8g2.setFont(u8g2_font_6x13_t_cyrillic);

    u8g2.setCursor(0, 15);
    u8g2.print(line1);

    u8g2.setCursor(0, 27);
    u8g2.print(line2);

  } while (u8g2.nextPage());
}

void setup()
{
  lastTime = millis();
  Serial.begin(115200);
  while (!Serial);
  Serial.println("OLED Test");
  pinMode(VEXT_PIN, OUTPUT);
  digitalWrite(VEXT_PIN, LOW);
  u8g2.begin();
  u8g2.enableUTF8Print();
  sensor.begin();

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // RADIO
  int state = radio.begin(868.7, 125.0, 12, 5);
  if (state != RADIOLIB_ERR_NONE)
  {
    printText("ERROR radio.begin", lastText);
  }
  radio.setPacketReceivedAction(setFlag);
  radio.startReceive();
}

/**************************
    SLAVE LOOP
 *************************/
#ifdef SLAVE_MODE
void loop()
{
  sensor.requestTemperatures();
  String temperature = String(sensor.getTempCByIndex(0));
  if (interruptFlag)
  {
    interruptFlag = false;
    uint16_t status = radio.getIrqStatus();

    if (status & RADIOLIB_SX126X_IRQ_TX_DONE)
    {
      Serial.println("Ответ успешно улетел Мастеру!");
      printText("Ответ отправлен", lastText);

      radio.startReceive();
      Serial.println("Слейв переведен в режим приема");
    }
    else if (status & RADIOLIB_SX126X_IRQ_RX_DONE)
    {
      Package rxPacket;

      int16_t state = radio.readData((uint8_t *)&rxPacket, sizeof(Package));

      if (state == RADIOLIB_ERR_NONE)
      {
        if (rxPacket.from_id == MASTER_ID && rxPacket.type_package == request)
        {

          Serial.println("Получен запрос! Формирую ответ...");
          printText("Запрос! Отвечаю", lastText);

          Package txPacket;
          txPacket.from_id = SLAVE_ID;
          txPacket.to_id = MASTER_ID;
          txPacket.type_package = response;
          sensor.requestTemperatures();
          float floatTemp = sensor.getTempCByIndex(0);
          txPacket.temperature = int16_t(floatTemp * 100);

          int state_tx = radio.startTransmit((uint8_t *)&txPacket, sizeof(Package));
          if (state_tx == RADIOLIB_ERR_NONE)
          {
            Serial.println("Начинаю отправку ответа...");
          }
          else
          {
            Serial.print("Ошибка отправки ответа: ");
            Serial.println(state_tx);

            radio.startReceive();
          }
        }
        else
        {
          Serial.println("Получен пакет не от мастера или не запрос");
          printText("Пакет не наш", lastText);

          radio.startReceive();
        }
      }
      else
      {
        Serial.print("Ошибка получения пакета: ");
        Serial.println(state);
        printText("Ошибка получения пакета!", lastText);

        radio.startReceive();
      }
    }
  }

  //   if(temperature == "-127.00") {
  //     printText("Error!", lastText);
  //   } else
  if (millis() - lastTime >= 100)
  {
    lastTime = millis();
    printText(temperature, lastText);
  }
}
#endif

/**************************
    MASTER LOOP
 *************************/

#ifdef MASTER_MODE
enum MASTER_STATE
{
  WAIT_FOR_CLICK,
  WAIT_FOR_RESPONSE,
  WAIT_FOR_TX_DONE
};
MASTER_STATE code_state = WAIT_FOR_CLICK;
unsigned long requestTime;

void loop()
{
  static unsigned long lastPresssed = 0;
  static bool buttonPressed = 0;
  if (code_state == WAIT_FOR_CLICK)
  {
    printTwoLines("Нажмите кнопку для", "запроса телеметрии", lastText);
    if (buttonHold(lastPresssed, buttonPressed, 2000, 5000, BUTTON_PIN))
    {
      printText("Выключениe...!", lastText);
      delay(1000);
      deepSleepOn(radio);
    }
    if (buttonFastPressed(100, 2000, BUTTON_PIN))
    {

      printText("Отправляю запрос...", lastText);
      Serial.println("Кнопка нажата! Отправка пакета...");

      Package txPacket;
      txPacket.from_id = MASTER_ID;
      txPacket.to_id = SLAVE_ID;
      txPacket.type_package = request;
      txPacket.temperature = 0;

      uint16_t state = radio.startTransmit((uint8_t *)&txPacket, sizeof(Package));

      if (state == RADIOLIB_ERR_NONE)
      {
        Serial.println("Пакет начал отправку. Жду завершения TX.");
        requestTime = millis();
        code_state = WAIT_FOR_TX_DONE;
      }
      else
      {
        printText("Ошибка отправки", lastText);
        Serial.print("Код ошибки LoRa: ");
        Serial.println(state);
        delay(2000);
      }

      while (digitalRead(BUTTON_PIN) == LOW);
    }
  }
  else if (code_state == WAIT_FOR_TX_DONE)
  {
    if (interruptFlag)
    {
      interruptFlag = false;
      uint16_t status = radio.getIrqStatus();

      if (status & RADIOLIB_SX126X_IRQ_TX_DONE)
      {
        Serial.println("Пакет успешно отправлен! Включаю приемник.");
        printTwoLines("Пакет отправлен", "Жду ответа", lastText);
        radio.startReceive();
        requestTime = millis();
        code_state = WAIT_FOR_RESPONSE;
      }
    }

    if (millis() - requestTime >= 5000)
    {
      Serial.println("Таймаут передачи!");
      printText("Ошибка передачи", lastText);
      radio.standby();
      delay(2000);
      code_state = WAIT_FOR_CLICK;
    }
  }

  else if (code_state == WAIT_FOR_RESPONSE)
  {
    if (interruptFlag)
    {
      interruptFlag = false;
      uint16_t status = radio.getIrqStatus();

      if (status & RADIOLIB_SX126X_IRQ_RX_DONE)
      {
        Package rxPacket;
        uint16_t state = radio.readData((uint8_t *)&rxPacket, sizeof(Package));

        if (state == RADIOLIB_ERR_NONE)
        {

          if (rxPacket.from_id == SLAVE_ID && rxPacket.type_package == response)
          {
            Serial.print("Ответ получен! Температура: ");
            Serial.println(rxPacket.temperature);

            String msg = "Темп: " + String(rxPacket.temperature / 100.0) + " C";
            printTwoLines("Данные приняты!", msg, lastText);

            delay(5000);
            code_state = WAIT_FOR_CLICK;
          }
          else
          {

            Serial.println("Получен чужой пакет. Игнорируем.");
            radio.startReceive();
          }
        }
        else
        {
          printText("Пакет ошибочный", lastText);
          Serial.println("Ошибка декодирования пакета.");
          delay(2000);
          code_state = WAIT_FOR_CLICK;
        }
      }
    }
    if (millis() - requestTime >= TIMEOUT_TIME)
    {
      Serial.println("Таймаут! Слейв не отвечает.");
      printTwoLines("Ошибка связи:", "пакет не пришёл", lastText);

      radio.standby();
      delay(3000);
      code_state = WAIT_FOR_CLICK;
    }
  }
}
#endif