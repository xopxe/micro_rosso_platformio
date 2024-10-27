#ifndef __env_bme680_h
#define __env_bme680_h

#include <Wire.h>

#define BME680_TOPIC_TEMPERATURE "/internal/temperature"
#define BME680_TOPIC_HUMIDITY "/internal/humidity"
#define BME680_TOPIC_PRESSURE "/internal/pressure"
#define BME680_TOPIC_GAS_RESISTANCE "/internal/gas_resistance"

class EnvBME680 {
public:
  EnvBME680();
  static bool setup(TwoWire &wire = Wire ); 
};

#endif  // __env_bme680_h
