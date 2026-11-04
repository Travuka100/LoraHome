// ============================================================================
//  RECEIVER (приёмник) — Heltec WiFi LoRa 32 V3, питание от LiPo
//
//   - короткое нажатие кнопки -> запрос телеметрии у передатчика;
//   - надёжность: до MAX_ATTEMPTS ре-трансляций со свежим seq и бэкоффом,
//     ответ валидируется CRC/адресами, дубликаты отсекаются по (from_id, seq);
//   - на экране: температуры D1/D2, RSSI и ЗАРЯД БАТАРЕИ;
//   - долгое нажатие (>= BTN_LONG_MS) -> глубокий сон;
//   - авто-сон после IDLE_SLEEP_MS бездействия (энергосбережение);
//   - выход из сна: press-and-hold >= BTN_LONG_MS (защита от ложных
//     пробуждений в кармане); короткое нажатие после сна игнорируется.
//
//  Кнопка включена в схему "выключение" по требованию ТЗ только здесь —
//  передатчик работает от сети и кнопочных функций не имеет.
// ============================================================================

#include "common.h"
#include <esp_sleep.h>

// Global objects declared extern in common.h (defined once, in this TU).
SX1262 fw::radio = SX1262(new Module(board::LORA_NSS, board::LORA_DIO1,
                                     board::LORA_NRST, board::LORA_BUSY));
U8G2_SSD1306_128X64_NONAME_1_SW_I2C
    fw::display(U8G2_R0, board::OLED_SCL, board::OLED_SDA, board::OLED_RST);

namespace {

board::Button g_button(board::BUTTON_PIN, fw::BTN_DEBOUNCE_MS,
                       fw::BTN_LONG_MS);

uint8_t g_seq = 0; // rolling request sequence number

// Last successfully received telemetry (kept for the status screen).
bool g_haveData = false;
lorahome::Packet g_data{};
int16_t g_rssi = 0;
uint32_t g_dataAgeStartMs = 0;

uint32_t g_lastActivityMs = 0;
uint32_t g_wakeGuardUntilMs = 0; // ignore short clicks right after wake
uint32_t g_batteryTickMs = 0;
uint16_t g_batteryMv = 0;

enum State : uint8_t { IDLE, SHOW_RESULT };
State g_state = IDLE;
uint32_t g_resultShownMs = 0;

// --- helpers ------------------------------------------------------------------
bool wokeFromSleep() {
  return esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0;
}

void markActivity() { g_lastActivityMs = millis(); }

float tempOrInvalid(int16_t raw) { return raw / 100.0f; }

void updateUi() {
  char l1[40], l2[40], l3[40];

  const uint8_t pct = board::batteryPercent(g_batteryMv);

  if (g_state == SHOW_RESULT && g_haveData) {
    const uint32_t ageS = (millis() - g_dataAgeStartMs) / 1000;

    if (g_data.flags & lorahome::FLAG_TEMP1_VALID) {
      snprintf(l1, sizeof(l1), "D1: %5.1f C",
               static_cast<double>(tempOrInvalid(g_data.temp1)));
    } else {
      strlcpy(l1, "D1: --.- C (нет)", sizeof(l1));
    }
    if (g_data.flags & lorahome::FLAG_TEMP2_VALID) {
      snprintf(l2, sizeof(l2), "D2: %5.1f C",
               static_cast<double>(tempOrInvalid(g_data.temp2)));
    } else {
      strlcpy(l2, "D2: --.- C (нет)", sizeof(l2));
    }
    snprintf(l3, sizeof(l3), "Бат:%d%% Rssi:%d %lus", pct, g_rssi,
             static_cast<unsigned long>(ageS));
    fw::showLines(l1, l2, l3);
  } else {
    strlcpy(l1, "Приёмник LoraHome", sizeof(l1));
    if (!g_haveData) {
      strlcpy(l2, "Кнопка: запрос", sizeof(l2));
    } else {
      snprintf(l2, sizeof(l2), "Последний сбой связи");
    }
    snprintf(l3, sizeof(l3), "Бат: %d%% (%umV)", pct, g_batteryMv);
    fw::showLines(l1, l2, l3);
  }
}

// One full request/response round with retransmissions.
// Returns true if a valid response was captured into g_data.
bool doTelemetryRequest() {
  fw::showLines("Запрос телеметрии...", "жду ответа", nullptr);

  for (uint8_t attempt = 1; attempt <= fw::MAX_ATTEMPTS; ++attempt) {
    // Retries keep the most robust PHY (SF12/4/5 = max link budget) but use
    // a fresh sequence number and a randomized backoff, so a repeated frame
    // lost to noise or collision still gets an independent chance.
    if (attempt > 1) {
      delay(fw::RETRY_BACKOFF_MS * attempt + (micros() & 0x7F));
    }

    lorahome::Packet req;
    lorahome::initPacket(req, fw::RECEIVER_ID, fw::SENDER_ID,
                         lorahome::TYPE_REQUEST, g_seq);
    lorahome::finalize(req);

    if (fw::transmitAndWait(req) != RADIOLIB_ERR_NONE) {
      Serial.print("TX failed, attempt ");
      Serial.println(attempt);
      continue; // radio re-armed inside transmitAndWait
    }

    // Wait for the response (bounded!), pumping the RX filter.
    const uint32_t deadline = millis() + fw::RESPONSE_TIMEOUT_MS;
    bool got = false;
    while (millis() < deadline) {
      if (fw::pollReceive(fw::SENDER_ID, lorahome::TYPE_RESPONSE, 250)) {
        lorahome::Packet resp;
        int16_t rssi = 0;
        if (fw::takeReceived(resp, &rssi) == RADIOLIB_ERR_NONE) {
          // Application-level ACK: response must echo our request's seq.
          if (resp.seq == req.seq) {
            g_data = resp;
            g_rssi = rssi;
            got = true;
            break;
          }
          Serial.println("Response seq mismatch (stale answer), ignored.");
        }
      }
      // Keep button responsive even during a long wait.
      g_button.tick();
    }

    if (got) {
      ++g_seq; // next request gets a fresh sequence number
      g_haveData = true;
      g_dataAgeStartMs = millis();
      return true;
    }
    Serial.print("No valid response, attempt ");
    Serial.println(attempt);
  }
  ++g_seq; // burn the seq anyway so stale responses can't match later
  return false;
}

} // namespace

void setup() {
  if (fw::commonInit() != RADIOLIB_ERR_NONE) {
    return; // error screen already shown by commonInit
  }

  // Battery reading BEFORE anything else so the first frame shows real data.
  g_batteryMv = board::batteryMillivolts();
  g_batteryTickMs = millis();

  markActivity();
  if (wokeFromSleep()) {
    // We were woken by a falling edge that may still be held. Require the
    // user to hold >= BTN_LONG_MS from now; swallow any click shorter than
    // that (prevents accidental requests / instant re-sleep in a pocket).
    g_wakeGuardUntilMs = millis() + fw::BTN_LONG_MS + 500;
    fw::showLines("Пробуждение...", "удерживайте кнопку", nullptr);
  } else {
    fw::showLines("Приёмник LoraHome", "Кнопка: запрос", nullptr);
  }
}

void loop() {
  g_button.tick();

  // --- periodic battery refresh (every 10 s) ---------------------------------
  if (millis() - g_batteryTickMs >= 10000) {
    g_batteryTickMs = millis();
    g_batteryMv = board::batteryMillivolts();
  }

  const board::ButtonEvent ev = g_button.event();
  const bool inWakeGuard = millis() < g_wakeGuardUntilMs;

  switch (ev) {
  case board::ButtonEvent::LONG:
    // Power off: deep sleep until the next press-and-hold.
    fw::showLines("Выключение...", "глубокий сон", nullptr);
    delay(300); // let the frame flush before cutting the rail
    board::deepSleepWithRadioOff(fw::radio); // never returns
    break;

  case board::ButtonEvent::SHORT:
    if (inWakeGuard) break; // accidental click right after wake -> ignore
    markActivity();
    if (doTelemetryRequest()) {
      g_state = SHOW_RESULT;
      g_resultShownMs = millis();
      updateUi();
    } else {
      g_haveData = false; // last data considered stale after failed session
      g_state = IDLE;
      fw::showLines("Связь не установлена", "передатчик не отвечает",
                    "повторите позже");
      delay(1500);
      updateUi();
    }
    break;

  case board::ButtonEvent::NONE:
  default:
    break;
  }

  // Result screen auto-falls-back to idle status after DISPLAY_HOLD_MS.
  if (g_state == SHOW_RESULT &&
      millis() - g_resultShownMs >= fw::DISPLAY_HOLD_MS) {
    g_state = IDLE;
  }

  updateUi();

  // --- automatic power save ----------------------------------------------------
  if (g_state == IDLE && !g_button.pressed() &&
      millis() - g_lastActivityMs >= fw::IDLE_SLEEP_MS) {
    fw::showLines("Авто-сон...", nullptr, nullptr);
    delay(300);
    board::deepSleepWithRadioOff(fw::radio); // never returns
  }

  delay(20); // ~50 Hz UI/button loop; radio IRQs are handled via flag polls
}
