#include "common.h"

namespace fw {

static volatile bool s_interruptFlag = false;

static void IRAM_ATTR onDio1_isr() { s_interruptFlag = true; }

// --- Display ------------------------------------------------------------------
// Layout of the 128x64 SSD1306 (U8g2 "F" = full-buffer, no flicker):
//
//   +------------------------------+
//   | Title              [_|__] 85%|  <- header row, battery icon top-right
//   |------------------------------|  <- thin separator line
//   | body line 1                    |
//   | body line 2                    |
//   | body line 3                    |
//   +------------------------------+
//
// Redraws are change-deduplicated to spare the OLED and the CPU.
static char s_lastTitle[24] = "";
static char s_lastL1[40] = "";
static char s_lastL2[40] = "";
static char s_lastL3[40] = "";
static uint8_t s_lastBatt = 0xFF; // 0xFF = not drawn yet

// Battery pictogram: 18x9 body + 2x3 cap, drawn at (x, y) top-left corner.
static void drawBatteryIcon(uint8_t pct) {
  const int x = 128 - 18 - 2; // right margin 2 px
  const int y = 2;
  display.drawFrame(x, y, 18, 9);           // body outline
  display.drawFilledFrame(x + 18, y + 3, 2, 3); // positive terminal nub

  const int innerW = 16; // drawable area inside the frame
  const int filled = (innerW * static_cast<int>(pct)) / 100;
  if (filled > 0) {
    display.drawFilledFrame(x + 1, y + 1, filled, 7);
  }
}

void uiDraw(const char *title, uint8_t battPct, const char *l1, const char *l2,
            const char *l3) {
  if (title == nullptr) title = s_lastTitle; // keep previous header
  if (battPct == 0xFF) battPct = s_lastBatt; // keep previous battery value
  if (l1 == nullptr) l1 = "";
  if (l2 == nullptr) l2 = "";
  if (l3 == nullptr) l3 = "";

  if (strcmp(title, s_lastTitle) == 0 && strcmp(l1, s_lastL1) == 0 &&
      strcmp(l2, s_lastL2) == 0 && strcmp(l3, s_lastL3) == 0 &&
      battPct == s_lastBatt) {
    return; // nothing changed -> no SPI traffic, no flicker
  }
  strlcpy(s_lastTitle, title, sizeof(s_lastTitle));
  strlcpy(s_lastL1, l1, sizeof(s_lastL1));
  strlcpy(s_lastL2, l2, sizeof(s_lastL2));
  strlcpy(s_lastL3, l3, sizeof(s_lastL3));
  s_lastBatt = battPct;

  display.clearBuffer();

  // Header: bold-ish title on the left, battery in the TOP-RIGHT corner.
  display.setFont(u8g2_font_helvB10_t_cyrillic);
  display.setCursor(0, 12);
  display.print(s_lastTitle);
  drawBatteryIcon(s_lastBatt);
  display.drawLine(0, 15, 127, 15); // separator

  // Body lines.
  display.setFont(u8g2_font_6x13_t_cyrillic);
  display.setCursor(0, 30);
  display.print(s_lastL1);
  if (s_lastL2[0]) {
    display.setCursor(0, 45);
    display.print(s_lastL2);
  }
  if (s_lastL3[0]) {
    display.setCursor(0, 60);
    display.print(s_lastL3);
  }
  display.sendBuffer();
}

void showError(const char *what, int code) {
  char buf[40];
  snprintf(buf, sizeof(buf), "%s err:%d", what, code);
  s_lastTitle[0] = '\0'; // force redraw
  uiDraw("ОШИБКА", 0xFF, buf, "проверьте", "подключение");
  Serial.println(buf);
}

// --- Common init -------------------------------------------------------------
int commonInit() {
  Serial.begin(115200);

  pinMode(board::VEXT_PIN, OUTPUT);
  digitalWrite(board::VEXT_PIN, LOW); // peripheral rail ON

  display.begin();
  display.enableUTF8Print();
  display.clearDisplay();

  const int state = board::radioBegin(radio);
  if (state != RADIOLIB_ERR_NONE) {
    showError("radio.begin", state);
    return state;
  }

  radio.setDio1Action(onDio1_isr);
  radio.startReceive();
  Serial.println("Radio OK, RX armed.");
  return RADIOLIB_ERR_NONE;
}

// --- Radio helpers -------------------------------------------------------------
int waitForIrq(uint32_t timeoutMs) {
  noInterrupts();
  s_interruptFlag = false; // clear BEFORE polling to avoid lost edges
  interrupts();

  const uint32_t start = millis();
  while (!s_interruptFlag) {
    if (millis() - start >= timeoutMs) return -1;
    yield(); // keep RTOS/feeding happy; flag is set asynchronously by ISR
  }
  return static_cast<int>(radio.getIrqStatus());
}

// Check for an already-latched IRQ WITHOUT waiting. This closes the race where
// a packet arrives while the firmware is busy with SPI transactions (e.g.
// re-arming RX after the previous frame): DIO1 pulsed, we were not listening,
// but the IRQ status register still holds RX_DONE. Without this check such
// packets were silently dropped -> "transmitter never answers".
static int pollIrqNow() {
  // Read the latched SX1262 status register directly: it reports RX_DONE even
  // if the DIO1 edge fired while we were busy with SPI transactions.
  const int irq = static_cast<int>(radio.getIrqStatus());
  if (irq == 0) {
    // Radio idle -> consume any stale ISR flag so the subsequent bounded wait
    // isn't woken immediately by an old edge.
    noInterrupts();
    s_interruptFlag = false;
    interrupts();
  }
  return irq;
}

int transmitAndWait(const lorahome::Packet &pIn) {
  // RadioLib's API takes a non-const buffer; copy locally (16 bytes).
  lorahome::Packet p = pIn;
  const int st = radio.startTransmit(reinterpret_cast<uint8_t *>(&p),
                                     sizeof(p));
  if (st != RADIOLIB_ERR_NONE) return st;

  const int irq = waitForIrq(IRQ_WAIT_TX_MS);
  radio.startReceive(); // always re-arm the receiver afterwards

  if (irq < 0) return RADIOLIB_ERR_TX_TIMEOUT; // TX never completed
  if (irq & RADIOLIB_SX126X_IRQ_TX_DONE) return RADIOLIB_ERR_NONE;
  return RADIOLIB_ERR_UNKNOWN;                  // unexpected IRQ
}

// Internal single-slot mailbox filled by pollReceive().
static bool s_framePending = false;
static lorahome::Packet s_frame;
static int16_t s_frameRssi = 0;

bool pollReceive(uint8_t expectFrom, uint8_t expectType, uint32_t sliceMs) {
  // Fast path: an RX_DONE may have latched in the radio status register while
  // we were busy elsewhere (OLED refresh, SPI re-arm window). Consume it
  // immediately instead of waiting for a new DIO1 edge.
  int irq = pollIrqNow();
  if (!(irq & RADIOLIB_SX126X_IRQ_RX_DONE)) {
    // Nothing pending -> wait up to sliceMs for a fresh IRQ edge.
    irq = waitForIrq(sliceMs);
  }
  if (irq < 0) return false; // nothing arrived in this slice

  if (!(irq & RADIOLIB_SX126X_IRQ_RX_DONE)) {
    radio.startReceive(); // spurious/timeout IRQ -> keep listening
    return false;
  }

  uint8_t buf[sizeof(lorahome::Packet)];
  const int st = radio.readData(buf, sizeof(buf));
  if (st != RADIOLIB_ERR_NONE) {
    radio.startReceive();
    return false;
  }

  // Layered validation: length + magic + version + software CRC.
  // (Hardware CRC already filtered most corruption at the PHY.)
  lorahome::Packet pkt;
  if (!lorahome::parse(buf, sizeof(buf), pkt)) {
    Serial.println("RX frame failed validation, ignored.");
    radio.startReceive();
    return false;
  }
  // Addressing filter: drop packets not meant for us / wrong type.
  if (pkt.from_id != expectFrom || pkt.type != expectType) {
    Serial.println("Foreign packet ignored.");
    radio.startReceive();
    return false;
  }

  s_frame = pkt;
  s_frameRssi = radio.getRSSI();
  s_framePending = true;
  radio.startReceive();
  return true;
}

int takeReceived(lorahome::Packet &out, int16_t *rssiOut) {
  if (!s_framePending) return RADIOLIB_ERR_RX_TIMEOUT;
  out = s_frame;
  if (rssiOut != nullptr) *rssiOut = s_frameRssi;
  s_framePending = false;
  return RADIOLIB_ERR_NONE;
}

} // namespace fw
