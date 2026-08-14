#ifndef DS18B20_H
#define DS18B20_H

#include <OneWire.h>
#include <DallasTemperature.h>

DallasTemperature ds18b20_init(uint8_t pin);

#endif