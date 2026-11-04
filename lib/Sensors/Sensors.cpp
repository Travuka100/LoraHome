#include "Sensors.h"

Sensors::Sensors(uint8_t pin)
    : _oneWire(pin), _sensors(&_oneWire) {}

uint8_t Sensors::begin() {
  _sensors.begin();
  _deviceCount = _sensors.getDeviceCount();
  if (_deviceCount > MAX_SENSORS) {
    _deviceCount = MAX_SENSORS; // only the first two channels are reported
  }
  for (uint8_t i = 0; i < _deviceCount; ++i) {
    _sensors.getAddress(_addr[i], i);
    _valid[i] = false;
    _tempC[i] = 0.0f;
  }
  return _deviceCount;
}

void Sensors::refresh() {
  if (_deviceCount == 0) return;

  // One broadcast conversion serves all sensors on the bus in parallel.
  _sensors.setWaitForConversion(false);
  _sensors.requestTemperatures();
  _sensors.setWaitForConversion(true); // blocking wait (~750 ms @ 12 bit)

  for (uint8_t i = 0; i < _deviceCount; ++i) {
    const float t = _sensors.getTempC(_addr[i]);
    // DallasTemperature returns DEVICE_DISCONNECTED_C (-127) or NaN on error;
    // accept only values inside the DS18B20 spec range.
    if (isPlausible(t)) {
      _tempC[i] = t;
      _valid[i] = true;
    } else {
      _valid[i] = false; // keep last good value, flag channel as stale
    }
  }
}

bool Sensors::valid(uint8_t index) const {
  return index < _deviceCount && _valid[index];
}

float Sensors::temperatureC(uint8_t index) const {
  return index < _deviceCount ? _tempC[index] : 0.0f;
}
