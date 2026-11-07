// ============================================================================
//  RECEIVER (приёмник) — Heltec WiFi LoRa 32 V3, питание от LiPo
//
//  Простой, предсказуемый алгоритм (без наложений и «тихих» состояний):
//    короткое нажатие -> ОДИН запрос -> ждём ответ до RESPONSE_TIMEOUT_MS
//      - ответ принят          -> экран телеметрии;
//      - кнопки НЕ трогали     -> явно ОБЪЯВЛЯЕМ повтор: "Повтор 2/3";
//      - нажали кнопку         -> ОТМЕНА (никаких повторов);
//    долгое нажатие (>= 2 c)   -> выключение (глубокий сон).
//
//  Передатчик отвечает ОДИН раз на каждый новый seq. Поэтому повтор всегда
//  отправляется с НОВЫМ seq и только после полного окна ожидания — два
//  ответа на один запрос физически невозможны, дубликаты не нужны.
//
//  UI: главное меню — заголовок + две строки; заряд батареи — пиктограмма
//  в правом верхнем углу экрана (всегда, кроме передатчика).
// ============================================================================

#include "common.h"
#include <esp_sleep.h>

// Global objects declared extern in common.h (defined once, in this TU).
SX1262 fw::radio = SX1262(new Module(board::LORA_NSS, board::LORA_DIO1,
                                     board::LORA_NRST, board::LORA_BUSY));
U8G2_SSD1306_128X64_NONAME_F_SW_I2C
    fw::display(U8G2_R0, board::OLED_SCL, board::OLED_SDA, board::OLED_RST);

namespace {

board::Button g_button(board::BUTTON_PIN, fw::BTN_DEBOUNCE_MS,
                       fw::BTN_LONG_MS);

uint8_t g_seq = 0; // rolling request sequence number

bool g_haveData = false;
lorahome::Packet g_data{};
int16_t g_rssi = 0;
uint32_t g_dataAgeStartMs = 0;

uint32_t g_lastActivityMs = 0;
uint32_t g_wakeGuardUntilMs = 0;
uint32_t g_batteryTickMs = 0;
uint16_t g_batteryMv = 0;

enum State : uint8_t { MENU, SHOW_RESULT };
State g_state = MENU;
uint32_t g_resultShownMs = 0;

bool wokeFromSleep() {
  return esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0;
}

void markActivity() { g_lastActivityMs = millis(); }

uint8_t battPct() { return board::batteryPercent(g_batteryMv); }

void drawMenu() {
  if (g_haveData) {
    const uint32_t ageS = (millis() - g_dataAgeStartMs) / 1000;
    char l2[40];
    snprintf(l2, sizeof(l2), "Данные %lus назад",
             static_cast<unsigned long>(ageS));
    fw::uiDraw("LoraHome", battPct(), "Кратко: запрос данных", l2, nullptr);
  } else {
    fw::uiDraw("LoraHome", battPct(), "Кратко: запрос данных",
               "Долго: выключение", nullptr);
  }
}

void drawResult() {
  char l1[40], l2[40], l3[40];
  // Trust the validity FLAGS (single source of truth set by the sender's
  // buildResponse), not raw sentinel values: flag -> number, no flag -> "--.-".
  if (g_data.flags & lorahome::FLAG_TEMP1_VALID)
    snprintf(l1, sizeof(l1), "T1: %5.1f C",
             static_cast<double>(lorahome::decodeTempC(g_data.temp1)));
  else
    strlcpy(l1, "T1: --.- C", sizeof(l1));
  if (g_data.flags & lorahome::FLAG_TEMP2_VALID)
    snprintf(l2, sizeof(l2), "T2: %5.1f C",
             static_cast<double>(lorahome::decodeTempC(g_data.temp2)));
  else
    strlcpy(l2, "T2: --.- C", sizeof(l2));
  snprintf(l3, sizeof(l3), "RSSI %d dBm", g_rssi);
  fw::uiDraw("Телеметрия", battPct(), l1, l2, l3);
}

void updateUi() {
  if (g_state == SHOW_RESULT && g_haveData) drawResult();
  else drawMenu();
}

// One full user session: send ONE request, wait for its answer.
// Retries are announced on screen and only happen after the previous
// response window FULLY expired (so a late first answer can never overlap
// with the second request). Button press during the session cancels it.
void doTelemetryRequest() {
  g_batteryMv = board::batteryMillivolts();
  g_batteryTickMs = millis();

  bool got = false;
  bool cancelled = false;

  for (uint8_t attempt = 1; attempt <= fw::MAX_ATTEMPTS && !got && !cancelled;
       ++attempt) {
    // Announce EVERY transmission attempt, including retries.
    char l1[40];
    if (attempt == 1) strlcpy(l1, "Запрос 1 из 3...", sizeof(l1));
    else snprintf(l1, sizeof(l1), "Повтор %d из 3...", attempt);
    fw::uiDraw("LoraHome", battPct(), l1, "Кнопка - отмена", nullptr);

    // Fresh seq per attempt: one sender answer per unique seq.
    lorahome::Packet req;
    lorahome::initPacket(req, fw::RECEIVER_ID, fw::SENDER_ID,
                         lorahome::TYPE_REQUEST, g_seq);
    lorahome::finalize(req);

    if (fw::transmitAndWait(req) != RADIOLIB_ERR_NONE) {
      Serial.print("TX failed, attempt ");
      Serial.println(attempt);
      delay(fw::RETRY_BACKOFF_MS);
      continue;
    }

    // Single bounded wait loop: pump RX, watch the button, check deadline.
    const uint32_t deadline = millis() + fw::RESPONSE_TIMEOUT_MS;
    while (millis() < deadline) {
      if (fw::pollReceive(fw::SENDER_ID, lorahome::TYPE_RESPONSE, 150)) {
        lorahome::Packet resp;
        int16_t rssi = 0;
        if (fw::takeReceived(resp, &rssi) == RADIOLIB_ERR_NONE &&
            resp.seq == g_seq) {
          g_data = resp;
          g_rssi = rssi;
          got = true;
          break;
        }
        // Foreign/stale frame -> keep waiting inside the same window.
      }

      g_button.tick();
      switch (g_button.event()) {
      case board::ButtonEvent::LONG:
        fw::uiDraw("LoraHome", battPct(), "Выключение...", "глубокий сон",
                   nullptr);
        delay(300);
        board::deepSleepWithRadioOff(fw::radio); // never returns
      case board::ButtonEvent::SHORT:
        if (millis() < g_wakeGuardUntilMs) break;
        cancelled = true;
        break;
      case board::ButtonEvent::NONE:
      default:
        break;
      }
      if (cancelled) break;
    }

    if (cancelled) {
      fw::uiDraw("LoraHome", battPct(), "Отменено", nullptr, nullptr);
      delay(700);
    }
    // This seq is spent (answered or timed out) — burn it either way so a
    // hypothetical late duplicate of an old answer can never match later.
    ++g_seq;
    if (!got && !cancelled) delay(fw::RETRY_BACKOFF_MS);
  }

  markActivity();
  if (got) {
    g_haveData = true;
    g_dataAgeStartMs = millis();
    g_state = SHOW_RESULT;
    g_resultShownMs = millis();
    updateUi();
  } else if (!cancelled) {
    fw::uiDraw("LoraHome", battPct(), "Нет ответа", "передатчик молчит",
               nullptr);
    delay(1800);
    updateUi();
  }
}

} // namespace

void setup() {
  if (fw::commonInit() != RADIOLIB_ERR_NONE) {
    return; // error screen already shown by commonInit
  }

  g_batteryMv = board::batteryMillivolts();
  g_batteryTickMs = millis();

  markActivity();
  if (wokeFromSleep()) {
    // Woken by a falling edge that may still be held: require the user to
    // keep holding >= BTN_LONG_MS; swallow any shorter click.
    g_wakeGuardUntilMs = millis() + fw::BTN_LONG_MS + 500;
    fw::uiDraw("LoraHome", battPct(), "Пробуждение...",
               "удерживайте кнопку", nullptr);
  } else {
    drawMenu();
  }
}

void loop() {
  g_button.tick();

  if (millis() - g_batteryTickMs >= 10000) {
    g_batteryTickMs = millis();
    g_batteryMv = board::batteryMillivolts();
  }

  const bool inWakeGuard = millis() < g_wakeGuardUntilMs;

  switch (g_button.event()) {
  case board::ButtonEvent::LONG:
    fw::uiDraw("LoraHome", battPct(), "Выключение...", "глубокий сон",
               nullptr);
    delay(300);
    board::deepSleepWithRadioOff(fw::radio); // never returns
    break;

  case board::ButtonEvent::SHORT:
    if (inWakeGuard) break;
    doTelemetryRequest();
    break;

  case board::ButtonEvent::NONE:
  default:
    break;
  }

  if (g_state == SHOW_RESULT &&
      millis() - g_resultShownMs >= fw::DISPLAY_HOLD_MS) {
    g_state = MENU;
  }

  updateUi();

  if (g_state == MENU && !g_button.pressed() &&
      millis() - g_lastActivityMs >= fw::IDLE_SLEEP_MS) {
    fw::uiDraw("LoraHome", battPct(), "Авто-сон...", nullptr, nullptr);
    delay(300);
    board::deepSleepWithRadioOff(fw::radio); // never returns
  }

  delay(20);
}
