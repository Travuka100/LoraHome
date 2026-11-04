// ============================================================================
//  RECEIVER (приёмник) — Heltec WiFi LoRa 32 V3, питание от LiPo
//
//  UI:  главное меню — только заголовок + две строки; заряд батареи —
//       пиктограмма в ПРАВОМ ВЕРХНЕМ углу экрана (всегда).
//
//  Кнопка:
//   - короткое нажатие          -> запрос телеметрии у передатчика;
//   - нажатие ВО ВРЕМЯ запроса  -> ОТМЕНА (прекращаем ретрансляции);
//   - долгое нажатие (>= 2 c)   -> выключение (глубокий сон);
//   - пробуждение из сна        -> тоже долгое нажатие (защита от ложных).
//
//  Надёжность связи (простой алгоритм без лишних состояний):
//   - до MAX_ATTEMPTS одинаковых запросов с разными seq;
//   - каждый повтор ОБЪЯВЛЯЕТСЯ на экране ("Повтор 2/3");
//   - ответ проверяется: CRC + адреса + эхо seq + флаги валидности датчиков;
//   - дубликаты ответа отсекаются по (from_id, seq).
//
//  Передатчик работает от сети и кнопочных функций не имеет — они только тут.
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

// Last successfully received telemetry (kept for the result screen).
bool g_haveData = false;
lorahome::Packet g_data{};
int16_t g_rssi = 0;
uint32_t g_dataAgeStartMs = 0;

uint32_t g_lastActivityMs = 0;
uint32_t g_wakeGuardUntilMs = 0; // ignore short clicks right after wake
uint32_t g_batteryTickMs = 0;
uint16_t g_batteryMv = 0;

enum State : uint8_t { MENU, SHOW_RESULT };
State g_state = MENU;
uint32_t g_resultShownMs = 0;

// --- helpers ------------------------------------------------------------------
bool wokeFromSleep() {
  return esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0;
}

void markActivity() { g_lastActivityMs = millis(); }

uint8_t battPct() { return board::batteryPercent(g_batteryMv); }

void drawMenu() {
  if (g_haveData) {
    const uint32_t ageS = (millis() - g_dataAgeStartMs) / 1000;
    char l2[40];
    snprintf(l2, sizeof(l2), "Обновлено %lus назад",
             static_cast<unsigned long>(ageS));
    fw::uiDraw("LoraHome", battPct(), "Кратко: запрос данных", l2, nullptr);
  } else {
    fw::uiDraw("LoraHome", battPct(), "Кратко: запрос данных",
               "Долго: выключение", nullptr);
  }
}

void drawResult() {
  char l1[40], l2[40], l3[40];
  snprintf(l1, sizeof(l1), "T1: %5.1f C",
           static_cast<double>(g_data.temp1 / 100.0f));
  snprintf(l2, sizeof(l2), "T2: %5.1f C",
           static_cast<double>(g_data.temp2 / 100.0f));
  snprintf(l3, sizeof(l3), "RSSI %d dBm", g_rssi);
  fw::uiDraw("Телеметрия", battPct(), l1, l2, l3);
}

void updateUi() {
  if (g_state == SHOW_RESULT && g_haveData) {
    drawResult();
  } else {
    drawMenu();
  }
}

// One request/response session with retransmissions.
// Pressing the button during the session CANCELS it (returns false without
// marking data stale). Returns true on a valid response captured into g_data.
bool doTelemetryRequest() {
  // Fresh battery reading at the start of every user action.
  g_batteryMv = board::batteryMillivolts();
  g_batteryTickMs = millis();

  for (uint8_t attempt = 1; attempt <= fw::MAX_ATTEMPTS; ++attempt) {
    // Fresh seq per attempt: the sender suppresses duplicates by seq, so a
    // repeated frame with the SAME seq would be silently ignored there.
    lorahome::Packet req;
    lorahome::initPacket(req, fw::RECEIVER_ID, fw::SENDER_ID,
                         lorahome::TYPE_REQUEST, g_seq);
    lorahome::finalize(req);

    // Announce every retransmission on screen — the user must SEE that the
    // device is retrying, not silently waiting.
    char l1[40], l2[40];
    if (attempt == 1) {
      strlcpy(l1, "Запрос телеметрии...", sizeof(l1));
    } else {
      snprintf(l1, sizeof(l1), "Повтор %d/%d", attempt,
               static_cast<int>(fw::MAX_ATTEMPTS));
    }
    strlcpy(l2, "Кнопка - отмена", sizeof(l2));
    fw::uiDraw("LoraHome", battPct(), l1, l2, nullptr);

    if (fw::transmitAndWait(req) != RADIOLIB_ERR_NONE) {
      Serial.print("TX failed, attempt ");
      Serial.println(attempt);
      continue; // radio re-armed inside transmitAndWait
    }

    // Bounded wait for the response, pumping the RX filter and the button.
    const uint32_t deadline = millis() + fw::RESPONSE_TIMEOUT_MS;
    while (millis() < deadline) {
      if (fw::pollReceive(fw::SENDER_ID, lorahome::TYPE_RESPONSE, 150)) {
        lorahome::Packet resp;
        int16_t rssi = 0;
        if (fw::takeReceived(resp, &rssi) == RADIOLIB_ERR_NONE &&
            resp.seq == req.seq && // application ACK: response echoes our seq
            (resp.flags & (lorahome::FLAG_TEMP1_VALID |
                           lorahome::FLAG_TEMP2_VALID))) {
          g_data = resp;
          g_rssi = rssi;
          ++g_seq;
          g_haveData = true;
          g_dataAgeStartMs = millis();
          return true;
        }
        // Stale answer or "no valid sensors" reply -> ignore, keep waiting.
      }

      // Button handling inside the wait loop: LONG still means power off,
      // any press cancels the session.
      g_button.tick();
      switch (g_button.event()) {
      case board::ButtonEvent::LONG:
        fw::uiDraw("LoraHome", battPct(), "Выключение...", "глубокий сон",
                   nullptr);
        delay(300);
        board::deepSleepWithRadioOff(fw::radio); // never returns
        break;
      case board::ButtonEvent::SHORT:
        if (millis() < g_wakeGuardUntilMs) break; // swallow post-wake click
        Serial.println("Request cancelled by user.");
        fw::uiDraw("LoraHome", battPct(), "Отменено", nullptr, nullptr);
        delay(600);
        ++g_seq; // burn seq so a late answer can't match anything later
        return false;
      case board::ButtonEvent::NONE:
      default:
        break;
      }
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
    // Woken by a falling edge that may still be held: require the user to
    // keep holding >= BTN_LONG_MS; swallow any shorter click (prevents
    // accidental requests / instant re-sleep in a pocket).
    g_wakeGuardUntilMs = millis() + fw::BTN_LONG_MS + 500;
    fw::uiDraw("LoraHome", battPct(), "Пробуждение...",
               "удерживайте кнопку", nullptr);
  } else {
    drawMenu();
  }
}

void loop() {
  g_button.tick();

  // --- periodic battery refresh (every 10 s) ---------------------------------
  if (millis() - g_batteryTickMs >= 10000) {
    g_batteryTickMs = millis();
    g_batteryMv = board::batteryMillivolts();
  }

  const bool inWakeGuard = millis() < g_wakeGuardUntilMs;

  switch (g_button.event()) {
  case board::ButtonEvent::LONG:
    // Power off: deep sleep until the next press-and-hold.
    fw::uiDraw("LoraHome", battPct(), "Выключение...", "глубокий сон",
               nullptr);
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
      fw::uiDraw("LoraHome", battPct(), "Передатчик не отвечает",
                 "повторите позже", nullptr);
      delay(1500);
      updateUi();
    }
    break;

  case board::ButtonEvent::NONE:
  default:
    break;
  }

  // Result screen auto-falls-back to the menu after DISPLAY_HOLD_MS.
  if (g_state == SHOW_RESULT &&
      millis() - g_resultShownMs >= fw::DISPLAY_HOLD_MS) {
    g_state = MENU;
  }

  updateUi();

  // --- automatic power save ---------------------------------------------------
  if (g_state == MENU && !g_button.pressed() &&
      millis() - g_lastActivityMs >= fw::IDLE_SLEEP_MS) {
    fw::uiDraw("LoraHome", battPct(), "Авто-сон...", nullptr, nullptr);
    delay(300);
    board::deepSleepWithRadioOff(fw::radio); // never returns
  }

  delay(20); // ~50 Hz UI/button loop; radio IRQs are handled via flag polls
}
