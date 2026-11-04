#ifndef BOARD_H
#define BOARD_H

#include <Arduino.h>
#include <RadioLib.h>

// ============================================================================
//  Shared board-level helpers for Heltec WiFi LoRa 32 V3 (ESP32 + SX1262)
//  used by both firmware roles (sender / receiver).
// ============================================================================

namespace board {

// --- Pin mapping (Heltec WiFi LoRa 32 V3 / ESP32-SX1262) -------------------
constexpr uint8_t BUTTON_PIN = 0; // front button, active LOW (INPUT_PULLUP)
constexpr uint8_t VEXT_PIN = 36;  // enables 3V3 to OLED/LoRa periphery
constexpr uint8_t BAT_ADC_PIN = 37; // battery voltage divider (LiPo)
constexpr uint8_t OLED_SCL = 18;
constexpr uint8_t OLED_SDA = 17;
constexpr uint8_t OLED_RST = 21;
constexpr uint8_t LORA_NSS = 8;
constexpr uint8_t LORA_DIO1 = 14;
constexpr uint8_t LORA_NRST = 12;
constexpr uint8_t LORA_BUSY = 13;
constexpr uint8_t ONEWIRE_PIN = 4;

// --- LoRa PHY (must be identical on both ends!) -----------------------------
constexpr float LORA_FREQ = 868.7f;
constexpr float LORA_BW = 125.0f;
constexpr uint8_t LORA_SF = 12;
constexpr int8_t LORA_CR = 5;     // coding rate 4/5
constexpr int8_t LORA_POWER = 14; // dBm
constexpr uint8_t LORA_PREAMBLE = 8;

// --- Button events ----------------------------------------------------------
enum class ButtonEvent : uint8_t {
  NONE,
  SHORT, // released before the long-press threshold
  LONG   // held at least LONG_MS (debounced), regardless of release
};

// Non-blocking button state machine with hysteresis:
//  - a press shorter than LONG_MS produces SHORT once on release;
//  - holding >= LONG_MS produces LONG exactly once (key repeat suppressed);
//  - contact bounce (< DEBOUNCE_MS) is filtered out.
class Button {
public:
  Button(uint8_t pin, uint16_t debounceMs, uint16_t longMs);
  void tick();         // call as often as possible
  ButtonEvent event(); // returns and consumes a pending event
  bool pressed() const { return _pressed; }
  uint32_t heldMs() const;

private:
  const uint8_t _pin;
  const uint16_t _debounceMs;
  const uint16_t _longMs;
  bool _pressed = false;
  bool _longFired = false;
  uint32_t _lastChangeMs = 0;
  uint32_t _pressStartMs = 0;
  ButtonEvent _pending = ButtonEvent::NONE;
};

// --- Radio ------------------------------------------------------------------
// Bring up SX1262 with fixed, validated parameters + hardware CRC.
// Returns RADIOLIB_ERR_NONE on success, otherwise the RadioLib error code.
int radioBegin(SX1262 &radio);

// --- Battery ----------------------------------------------------------------
// LiPo voltage via ADC2 (works while WiFi is off). Returns millivolts.
uint16_t batteryMillivolts();
// Rough charge percent from a LiPo discharge curve (0..100).
uint8_t batteryPercent(uint16_t mv);

// --- Deep sleep -------------------------------------------------------------
// Put radio + SPI + OLED peripheral rail into the lowest-power state and
// enter ESP32 deep sleep. Wakes on BUTTON_PIN falling edge (button press).
// Never returns.
void deepSleepWithRadioOff(SX1262 &radio);

} // namespace board

#endif // BOARD_H
