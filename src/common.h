#ifndef COMMON_H
#define COMMON_H

#include <Arduino.h>
#include <RadioLib.h>
#include <U8g2lib.h>
#include <LoraHomeProtocol.h>
#include "Board.h"

// ============================================================================
//  Common firmware services shared by SENDER and RECEIVER roles:
//   - single SX1262 instance + DIO1 interrupt plumbing,
//   - OLED text output with change-deduplication,
//   - robust RX/TX helpers (timeout-bounded IRQ wait, full packet validation).
// ============================================================================

namespace fw {

extern SX1262 radio;
extern U8G2_SSD1306_128X64_NONAME_1_SW_I2C display;

// Node addressing (both ends must agree)
constexpr uint8_t RECEIVER_ID = 0x01; // handheld receiver (battery, button)
constexpr uint8_t SENDER_ID = 0x02;   // transmitter with 2x DS18B20 (mains)

// Timing constants (ms)
constexpr uint32_t IRQ_WAIT_TX_MS = 5000;      // max wait for TX_DONE
constexpr uint32_t RESPONSE_TIMEOUT_MS = 1500; // wait for response per attempt
                                               // (SF9 airtime ~0.4 s/hop -> ample)
constexpr uint8_t MAX_ATTEMPTS = 4;            // request retransmissions
constexpr uint32_t RETRY_BACKOFF_MS = 120;     // pause between retries
constexpr uint32_t DISPLAY_HOLD_MS = 5000;     // result screen hold
constexpr uint32_t IDLE_SLEEP_MS = 60000;      // auto deep sleep (receiver)
constexpr uint16_t BTN_DEBOUNCE_MS = 30;
constexpr uint16_t BTN_LONG_MS = 2000;         // long press -> power off

// --- Display ------------------------------------------------------------------
// Print up to 3 lines; redraws only when the content actually changed.
void showLines(const char *l1, const char *l2 = nullptr,
               const char *l3 = nullptr);
void showError(const char *what, int code); // fatal init error screen

// Shared bring-up: serial, VEXT rail, OLED, radio + DIO1 IRQ, RX armed.
// Returns RADIOLIB_ERR_NONE or the failing RadioLib code.
int commonInit();

// --- Radio helpers --------------------------------------------------------------
// Wait (bounded!) for a DIO1 edge and return the raw IRQ status word,
// or -1 on timeout. Clears the ISR flag atomically w.r.t. the interrupt.
int waitForIrq(uint32_t timeoutMs);

// Transmit a fully-built packet and wait for TX_DONE, then re-arm RX.
// Returns RADIOLIB_ERR_NONE on success, negative RadioLib code otherwise.
int transmitAndWait(const lorahome::Packet &p);

// Non-blocking RX pump: poll the radio IRQ for up to sliceMs. Returns true
// if an RX_DONE event occurred and a complete frame was read into the
// internal validated buffer (use takeReceived() to consume it).
bool pollReceive(uint8_t expectFrom, uint8_t expectType, uint32_t sliceMs);

// Copy the last frame captured by pollReceive() (with its RSSI).
// Returns RADIOLIB_ERR_NONE if a fresh frame is available, else
// RADIOLIB_ERR_RX_TIMEOUT (nothing pending). Consumes the pending state.
int takeReceived(lorahome::Packet &out, int16_t *rssiOut);

} // namespace fw

#endif // COMMON_H
