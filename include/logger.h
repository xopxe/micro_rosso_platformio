#ifndef __roslogger_h
#define __roslogger_h

#define LOGGER_TOPIC_LOG "/rosout"

#include <rcl_interfaces/msg/log.h>

class Logger {
public:
  Logger();
  static bool setup();

  static void log(const char* s,
                  const char* file = "",
                  const char* func = "",
                  uint32_t line = 0,
                  uint8_t level = rcl_interfaces__msg__Log__INFO);
};

#endif  // __roslogger_h
