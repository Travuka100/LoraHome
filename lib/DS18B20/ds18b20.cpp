#include <ds18b20.h>

DallasTemperature ds18b20_init(uint8_t pin) {
    const uint8_t oneWireBus = pin;     
    OneWire oneWire(oneWireBus);
    DallasTemperature sensors(&oneWire);
    return sensors;
}