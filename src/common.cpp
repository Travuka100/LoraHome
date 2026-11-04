#include "common.h"

namespace fw {

static volatile bool s_interruptFlag = false;

static void IRAM_ATTR onDio1_isr() { s_interruptFlag = true; }

// --- Display -----------------------------------------------------------------
static char s_lastL1[40] = "";
static char s_lastL2[40] = "";
static char s_lastL3[40] = "";

void showLines(const char *l1, const char *l2, const char *l3) {
  if (l1 == nullptr) l1 = "";
  if (l2 == nullptr) l2 = "";
  if (l3 == nullptr) l3 = "";

  if (strcmp(l1, s_lastL1) == 0 && strcmp(l2, s_lastL2) == 0 &&
      strcmp(l3, s_lastL3) == 0) {
    return; // no change -> no redraw (saves OLED wear & power)
  }
  strlcpy(s_lastL1, l1, sizeof(s_lastL1));
  strlcpy(s_lastL2, l2, sizeof(s_lastL2));
  strlcpy(s_lastL3, l3, sizeof(s_lastL3));

  display.clearDisplay();
  display.firstPage();
  do {
    display.setFont(u8g2_font_6x13_t_cyrillic);
    display.setCursor(0, 15);
    display.print(s_lastL1);
    if (s_lastL2[0]) {
      display.setCursor(0, 32);
      display.print(s_lastL2);
    }
    if (s_lastL3[0]) {
      display.setCursor(0, 49);
      display.print(s_lastL3);
    }
  } while (display.nextPage());
}

void showError(const char *what, int code) {
  char buf[40];
  snprintf(buf, sizeof(buf), "%s err:%d", what, code);
  s_lastL1[0] = '\0'; // force redraw
  showLines("FATAL", buf, "проверьте железо");
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

int transmitAndWait(const lorahome::Packet &pIn) {
  // RadioLib's API takes a non-const buffer; copy locally (16 bytes).
  lorahome::Packet p = pIn;
  const int st = radio.startTransmit(reinterpret_cast<uint8_t *>(&p),
                                     sizeof(p));
  if (st != RADIOLIB_ERR_NONE) return st;

  const int irq = waitForIrq(IRQ_WAIT_TX_MS);
  radio.startReceive(); // always re-arm the receiver afterwards

  if (irq < 0) return RADIOLIB_ERR_TX_TIMEOUT;      // TX never completed
  if (irq & RADIOLIB_SX126X_IRQ_TX_DONE) return RADIOLIB_ERR_NONE;
  return RADIOLIB_ERR_UNKNOWN;                        // unexpected IRQ
}

// Internal single-slot mailbox filled by pollReceive().
static bool s_framePending = false;
static lorahome::Packet s_frame;
static int16_t s_frameRssi = 0;

bool pollReceive(uint8_t expectFrom, uint8_t expectType, uint32_t sliceMs) {
  const int irq = waitForIrq(sliceMs);
  if (irq < 0) return false; // nothing arrived in this slice

  if (!(irq & RADIOLIB_SX126X_IRQ_RX_DONE)) {
    radio.startReceive(); // spurious IRQ -> keep listening
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
