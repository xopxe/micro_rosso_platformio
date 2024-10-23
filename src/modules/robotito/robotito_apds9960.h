#ifndef __robotito_apds9960
#define __robotito_apds9960

#define APDS9960_TOPIC_PROXIMITY "/apds/proximity"
#define APDS9960_TOPIC_COLOR_RAW "/apds/color_raw"

class Apds9960 {
public:
  Apds9960();
  static bool setup();
};

#endif  // __robotito_apds9960
