#include "Board.h"

#include <SPI.h>
#include <esp_sleep.h>

namespace board {

// ---------------------------------------------------------------------------
// Button
// ---------------------------------------------------------------------------
Button::Button(uint8_t pin, uint16_t debounceMs, uint16_t longMs)
    : _pin(pin), _debounceMs(debounceMs), _longMs(longMs) {
  pinMode(_pin, INPUT_PULLUP);
}

uint32_t Button::heldMs() const {
  return _pressed ? (millis() - _pressStartMs) : 0;
}

void Button::tick() {
  const uint32_t now = millis();
  const bool rawPressed = (digitalRead(_pin) == LOW); // active LOW

  if (rawPressed != _pressed) {
    // Ignore transitions shorter than the debounce window.
    if (now - _lastChangeMs < _debounceMs) return;
    _lastChangeMs = now;

    if (rawPressed) {
      _pressed = true;
      _longFired = false;
      _pressStartMs = now;
    } else {
      _pressed = false;
      // Release before LONG threshold -> short click event.
      if (!_longFired && (now - _pressStartMs) >= _debounceMs) {
        _pending = ButtonEvent::SHORT;
      }
    }
    return;
  }

  // Still held: fire LONG exactly once after the threshold.
  if (_pressed && !_longFired && (now - _pressStartMs) >= _longMs) {
    _longFired = true;
    _pending = ButtonEvent::LONG;
  }
}

ButtonEvent Button::event() {
  const ButtonEvent e = _pending;
  _pending = ButtonEvent::NONE;
  return e;
}

// ---------------------------------------------------------------------------
// Radio
// ---------------------------------------------------------------------------
int radioBegin(SX1262 &radio) {
  // Heltec V3 wiring: SX1262 DIO2 drives the RF switch and DIO3 powers the
  // TCXO. Without these two calls the PA/antenna path stays dead -> the
  // module "transmits" but nothing is radiated (classic "peer never answers"
  // symptom on this board).
  int state = radio.setDio2AsRfSwitch(true);
  if (state != RADIOLIB_ERR_NONE) return state;

  state = radio.setTCXO(3.4f); // 3.4 V TCXO supply via DIO3
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("setTCXO status %d\n", state);
    // Non-fatal on modules with plain crystal: continue bring-up anyway.
  }

  state = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR, LORA_POWER,
                      LORA_PREAMBLE);
  if (state != RADIOLIB_ERR_NONE) return state;

  // Maximum range profile explicitly, one setting at a time so a rejected
  // value can never silently break the whole configuration:
  (void)radio.setFrequency(LORA_FREQ);
  (void)radio.setBandwidth(LORA_BW);
  (void)radio.setSpreadingFactor(LORA_SF); // SF12 - most robust
  (void)radio.setCodingRate(LORA_CR);      // 4/5
  (void)radio.setPreambleLength(LORA_PREAMBLE);
  state = radio.setOutputPower(LORA_POWER); // 22 dBm (SX1262 max)
  if (state != RADIOLIB_ERR_NONE) return state;
  // Allow the PA to draw the current needed for 22 dBm (default OCP is 140 mA).
  (void)radio.setCurrentLimit(2.4f);

  // Explicit header + payload CRC => transceiver drops corrupted packets.
  state = radio.setCRC(true);
  if (state != RADIOLIB_ERR_NONE) return state;

  return RADIOLIB_ERR_NONE;
}

// ---------------------------------------------------------------------------
// Battery
// ---------------------------------------------------------------------------
// Heltec WiFi LoRa 32 V3 (ESP32-S3): the battery divider is gated by
// VBAT_CTRL = GPIO37 and its tap goes to GPIO1 (ADC1_CH0). Reading any other
// pin (e.g. 35/36/37 themselves) always yields ~0 mV -> "battery 0%".
// Procedure mirrors Heltec's own library (heltec_vbat()): drive 37 LOW to
// connect the 220k/100k divider, settle, sample, release the gate again.
constexpr uint16_t kEmptyMv = 3300;
constexpr uint16_t kFullMv = 4200;

uint16_t batteryMillivolts() {
  analogSetPinAttenuation(VBAT_ADC_PIN, ADC_11db);
  analogReadResolution(12);

  pinMode(VBAT_CTRL_PIN, OUTPUT);
  digitalWrite(VBAT_CTRL_PIN, LOW); // divider ON
  delay(5);                         // settling time per Heltec reference code

  // Take 9 samples, drop the extremes, average the rest (spike rejection).
  constexpr size_t N = 9;
  uint16_t samples[N];
  for (size_t i = 0; i < N; ++i) {
    samples[i] = static_cast<uint16_t>(analogReadMilliVolts(VBAT_ADC_PIN));
    delayMicroseconds(200);
  }

  digitalWrite(VBAT_CTRL_PIN, HIGH); // keep divider unpowered between reads
  pinMode(VBAT_CTRL_PIN, INPUT);

  // Simple insertion sort (n=9).
  for (size_t i = 1; i < N; ++i) {
    uint16_t v = samples[i];
    size_t j = i;
    while (j > 0 && samples[j - 1] > v) {
      samples[j] = samples[j - 1];
      --j;
    }
    samples[j] = v;
  }
  uint32_t acc = 0;
  for (size_t i = 2; i < N - 2; ++i) acc += samples[i]; // trimmed mean
  const float rawMv = static_cast<float>(acc) / (N - 4);
  const uint32_t mv = static_cast<uint32_t>(rawMv * BAT_DIVIDER);
  return static_cast<uint16_t>(mv > 5000 ? 5000 : mv); // clamp sanity
}

uint8_t batteryPercent(uint16_t mv) {
  if (mv <= kEmptyMv) return 0;
  if (mv >= kFullMv) return 100;
  // Piecewise-linear LiPo curve approximation (better than pure linear):
  // 3.30V=0%, 3.60V=20%, 3.80V=60%, 4.00V=90%, 4.20V=100%
  struct Point {
    uint16_t mv;
    uint8_t pct;
  };
  static constexpr Point curve[] = {{3300, 0}, {3600, 20}, {3800, 60},
                                    {4000, 90}, {4200, 100}};
  for (size_t i = 1; i < sizeof(curve) / sizeof(curve[0]); ++i) {
    if (mv <= curve[i].mv) {
      const uint16_t span = curve[i].mv - curve[i - 1].mv;
      const uint8_t pspan = curve[i].pct - curve[i - 1].pct;
      return static_cast<uint8_t>(curve[i - 1].pct +
                                  (pspan * (mv - curve[i - 1].mv)) / span);
    }
  }
  return 100;
}

// ---------------------------------------------------------------------------
// Deep sleep
// ---------------------------------------------------------------------------
void deepSleepWithRadioOff(SX1262 &radio) {
  // Ask the SX1262 to enter its own sleep mode while SPI is still alive.
  (void)radio.sleep();

  // Kill peripheral power rail (OLED + LoRa module supply on Heltec V3).
  digitalWrite(VEXT_PIN, HIGH);
  SPI.end();

  esp_sleep_enable_ext0_wakeup(static_cast<gpio_num_t>(BUTTON_PIN), 0);
  esp_deep_sleep_start(); // never returns
}

} // namespace board
