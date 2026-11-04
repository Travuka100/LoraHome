#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Reads up to two DS18B20 sensors on a single 1-Wire bus.
// Unlike the old lib/DS18B20 helper (which returned a DallasTemperature
// object pointing at a destroyed stack OneWire), all state here is owned
// by the Sensors instance and stays valid for its whole lifetime.
class Sensors {
public:
  static constexpr uint8_t MAX_SENSORS = 2;

  explicit Sensors(uint8_t pin);

  // Scan the bus, register found addresses. Returns number of sensors (0..2).
  uint8_t begin();

  // Blocking conversion + read of all registered sensors (~750 ms for 12 bit).
  // Marks each channel valid/invalid; invalid readings keep their previous
  // value but are flagged so the caller can show "--".
  void refresh();

  bool valid(uint8_t index) const;
  float temperatureC(uint8_t index) const; // only meaningful if valid()

  uint8_t count() const { return _deviceCount; }

private:
  static constexpr int16_t kScratchpadSize = 9;

  OneWire _oneWire;
  DallasTemperature _sensors;
  uint8_t _deviceCount = 0;
  DeviceAddress _addr[MAX_SENSORS] = {};
  float _tempC[MAX_SENSORS] = {};
  bool _valid[MAX_SENSORS] = {};

  static bool isPlausible(float t) { return t >= -55.0f && t <= 125.0f; }
};

#endif // SENSORS_H
