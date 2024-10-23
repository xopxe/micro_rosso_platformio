#ifndef __ticker_h
#define __ticker_h

#define TIMER_TICK_MS 1000  // 1Hz
#define TICKER_TOPIC_TICK "/tick"
#define TICKER_SERVICE_ADD "/add"

class Ticker {
public:
  Ticker();
  static bool setup();

  static timer_descriptor timer_tick;
};

#endif  // __ticker_h
