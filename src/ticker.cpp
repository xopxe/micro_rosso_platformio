#include "micro_rosso.h"

#include "ticker.h"
#include <std_msgs/msg/int32.h>
#include <rcl_interfaces/msg/log.h>

timer_descriptor Ticker::timer_tick;

static std_msgs__msg__Int32 msg_tick;

static publisher_descriptor pdescriptor_tick;

#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      return false;              \
    }                            \
  }
#define RCNOCHECK(fn)       \
  {                         \
    rcl_ret_t temp_rc = fn; \
    (void)temp_rc;          \
  }

Ticker::Ticker()
{
  msg_tick.data = 0;
};

static void tick_cb(int64_t last_call_time)
{
  // micro_rosso::set_timestamp(msg_tick.header.stamp);
  RCNOCHECK(rcl_publish(&pdescriptor_tick.publisher, &msg_tick, NULL));
  msg_tick.data++;
}

static void timer_handler_tick(rcl_timer_t *timer, int64_t last_call_time)
{
  for (int j = 0; j < Ticker::timer_tick.callbacks.size(); j++)
  {
    Ticker::timer_tick.callbacks[j](last_call_time);
  }
  return;
}

bool Ticker::setup(const char *topic_name)
{
  D_println("setup: ticker");

  pdescriptor_tick.qos = QOS_DEFAULT;
  pdescriptor_tick.type_support =
      (rosidl_message_type_support_t *)ROSIDL_GET_MSG_TYPE_SUPPORT(
          std_msgs, msg, Int32);
  pdescriptor_tick.topic_name = topic_name;
  micro_rosso::publishers.push_back(&pdescriptor_tick);

  // create a new timer
  Ticker::timer_tick.timeout_ns = RCL_MS_TO_NS(TIMER_TICK_MS);
  Ticker::timer_tick.timer_handler = timer_handler_tick;
  Ticker::timer_tick.callbacks.push_back(&tick_cb);
  micro_rosso::timers.push_back(&Ticker::timer_tick);

  return true;
}
