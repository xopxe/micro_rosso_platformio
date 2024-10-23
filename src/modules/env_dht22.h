#ifndef __env_dht22_h
#define __env_dht22_h

#define DHT22_TOPIC_TEMPERATURE "/internal/temperature"
#define DHT22_TOPIC_HUMIDITY "/internal/humidity"

class EnvDHT22 {
public:
  EnvDHT22();
  static bool setup();
};

#endif  // __env_dht22_h
