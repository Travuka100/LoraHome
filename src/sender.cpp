// ============================================================================
//  SENDER (передатчик) — Heltec WiFi LoRa 32 V3
//
//   - 2 x DS18B20 на одной 1-Wire шине (GPIO4);
//   - локальное отображение обеих температур на OLED;
//   - пассивный ответ на запрос телеметрии от приёмника (TYPE_REQUEST ->
//     TYPE_RESPONSE с эхом seq);
//   - питание от блока питания: НЕТ глубокого сна, НЕТ заряда батареи,
//     НЕТ кнопок выключения.
//
//  Ошибка датчика не блокирует работу: канал помечается невалидным
//  (TEMP_INVALID + флаг), второй датчик и связь продолжают работать.
// ============================================================================

#include "common.h"
#include <Sensors.h>

// Global objects declared extern in common.h
SX1262 fw::radio = SX1262(new Module(board::LORA_NSS, board::LORA_DIO1,
                                     board::LORA_NRST, board::LORA_BUSY));
U8G2_SSD1306_128X64_NONAME_F_SW_I2C
    fw::display(U8G2_R0, board::OLED_SCL, board::OLED_SDA, board::OLED_RST);

namespace {

Sensors g_sensors(board::ONEWIRE_PIN);

uint32_t g_lastRefreshMs = 0;
uint32_t g_lastDisplayMs = 0;
uint8_t g_lastSeqSeen = 0xFF; // last answered request seq (duplicate guard)

constexpr uint32_t SENSOR_PERIOD_MS = 2000;  // 12-bit conversion ~750 ms
constexpr uint32_t DISPLAY_PERIOD_MS = 1000; // UI refresh rate

void formatTemp(char *buf, size_t n, int idx) {
  if (g_sensors.valid(idx)) {
    snprintf(buf, n, "D%d: %5.1f C", idx + 1,
             static_cast<double>(g_sensors.temperatureC(idx)));
  } else {
    snprintf(buf, n, "D%d: --.- C", idx + 1);
  }
}

// battPct == kNoBattery (0xFE): the sender is mains powered — no battery UI.
void drawStatus(const char *note) {
  char l1[40], l2[40];
  if (g_sensors.count() == 0) {
    fw::uiDraw("Передатчик", 0xFE, "Нет датчиков 1-Wire!",
               "проверьте GPIO4/подтяжку", nullptr);
    return;
  }
  formatTemp(l1, sizeof(l1), 0);
  formatTemp(l2, sizeof(l2), 1);
  fw::uiDraw("Передатчик", 0xFE, l1, l2, note);
}

void handleRequest(const lorahome::Packet &req) {
  // The receiver increments seq on EVERY retry and only retries after its
  // response window fully expired, so a repeated SAME-seq frame here can
  // only be a multi-reception echo of one request (long cable/reflections).
  // Answer each NEW seq exactly once; ignore same-seq echoes to avoid TX
  // collisions with the answer we just sent.
  if (req.seq == g_lastSeqSeen) {
    Serial.println("Same-seq echo ignored.");
    return;
  }
  g_lastSeqSeen = req.seq;

  // Canonical response builder: encodes both channels and sets the validity
  // flags in ONE place (LoraHomeProtocol.h). Dead/absent channels become
  // TEMP_INVALID with no flag, so the receiver shows "--.-" for them ONLY.
  float temps[lorahome::MAX_SENSORS] = {NAN, NAN};
  for (uint8_t i = 0; i < lorahome::MAX_SENSORS; ++i) {
    if (g_sensors.valid(i)) temps[i] = g_sensors.temperatureC(i);
  }

  lorahome::Packet resp;
  lorahome::buildResponse(resp, fw::SENDER_ID, fw::RECEIVER_ID,
                          req.seq, temps); // echo seq = ACK

  const int st = fw::transmitAndWait(resp);
  if (st == RADIOLIB_ERR_NONE) {
    Serial.print("Answered seq ");
    Serial.print(req.seq);
    Serial.print(" T1=");
    Serial.print(resp.temp1);
    Serial.print(" T2=");
    Serial.print(resp.temp2);
    Serial.print(" flags=0x");
    Serial.println(resp.flags, HEX); // must be 0x03 with two live sensors
    drawStatus("Ответ отправлен");
    g_lastDisplayMs = millis();
  } else {
    Serial.print("Response TX failed: ");
    Serial.println(st);
    drawStatus("Ошибка ответа!");
    g_lastDisplayMs = millis();
  }
}

} // namespace

void setup() {
  if (fw::commonInit() != RADIOLIB_ERR_NONE) {
    // Radio dead: keep looping on the error screen (hardware fault).
    return;
  }

  const uint8_t found = g_sensors.begin();
  char buf[40];
  snprintf(buf, sizeof(buf), "Датчиков найдено: %d", found);
  Serial.println(buf);
  fw::uiDraw("Передатчик", 0xFE, buf, nullptr, nullptr);

  g_sensors.refresh(); // first synchronous reading
  g_lastRefreshMs = millis();
}

void loop() {
  // --- periodic sensor sampling (non-blocking w.r.t. radio events) ---------
  if (millis() - g_lastRefreshMs >= SENSOR_PERIOD_MS) {
    g_lastRefreshMs = millis();
    g_sensors.refresh();
  }

  // --- local display ---------------------------------------------------------
  if (millis() - g_lastDisplayMs >= DISPLAY_PERIOD_MS) {
    g_lastDisplayMs = millis();
    drawStatus("Ожидание запроса...");
  }

  // --- serve incoming requests ----------------------------------------------
  // Short poll slice keeps sensor sampling and UI responsive between frames.
  if (!fw::pollReceive(fw::RECEIVER_ID, lorahome::TYPE_REQUEST, 200)) return;

  lorahome::Packet pkt;
  int16_t rssi = 0;
  const int st = fw::takeReceived(pkt, &rssi);
  if (st == RADIOLIB_ERR_NONE) {
    Serial.print("Request received, RSSI: ");
    Serial.println(rssi);
    handleRequest(pkt);
  }
  // Invalid / foreign packets are silently dropped inside the RX pump.
}
