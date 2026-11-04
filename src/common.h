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
//   - pretty OLED UI (header + battery icon in the top-right corner),
//   - robust RX/TX helpers (timeout-bounded IRQ wait, full packet validation).
// ============================================================================

namespace fw {

extern SX1262 radio;
extern U8G2_SSD1306_128X64_NONAME_F_SW_I2C display;

// Node addressing (both ends must agree)
constexpr uint8_t RECEIVER_ID = 0x01; // handheld receiver (battery, button)
constexpr uint8_t SENDER_ID = 0x02;   // transmitter with 2x DS18B20 (mains)

// Timing constants (ms). LoRa PHY is SF12/125kHz: one 16-byte frame is on
// air ~1.1 s, so a full request->response round takes ~2.5 s. The response
// window MUST exceed the sender's blocking DS18B20 read (~0.75 s) + its TX
// time; 7 s gives comfortable margin. Retries happen ONLY after the window
// fully expired, always announced on screen ("Повтор 2 из 3").
constexpr uint32_t IRQ_WAIT_TX_MS = 8000;       // max wait for TX_DONE
constexpr uint32_t RESPONSE_TIMEOUT_MS = 7000;  // wait for response per attempt
constexpr uint8_t MAX_ATTEMPTS = 3;             // request transmissions total
constexpr uint32_t RETRY_BACKOFF_MS = 300;      // pause between retries
constexpr uint32_t DISPLAY_HOLD_MS = 15000;     // result screen hold
constexpr uint32_t IDLE_SLEEP_MS = 90000;       // auto deep sleep (receiver)
constexpr uint16_t BTN_DEBOUNCE_MS = 30;
constexpr uint16_t BTN_LONG_MS = 2000;          // long press -> power off

// --- Display ------------------------------------------------------------------
// Pretty 128x64 UI (change-deduplicated, no flicker):
// bold title on the left + battery pictogram in the TOP-RIGHT corner,
// separator line, up to three centered body lines.
// battPct == 0xFF keeps the previously drawn battery value.
void uiDraw(const char *title, uint8_t battPct, const char *l1,
            const char *l2, const char *l3);
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
