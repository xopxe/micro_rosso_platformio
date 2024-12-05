#ifndef __ticker_h
#define __ticker_h

#define TIMER_TICK_MS 1000 // 1Hz

class Ticker
{
public:
  Ticker();
  static bool setup(const char *topic_name = "/tick");

  static timer_descriptor timer_tick;
};

#endif // __ticker_h
