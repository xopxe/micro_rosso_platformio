#include "build_modules.h"
#ifdef BUILD_TICKER

#include "micro_rosso.h"

#include "ticker.h"
#include <std_msgs/msg/int32.h>
#include <example_interfaces/srv/add_two_ints.h>
#include <rcl_interfaces/msg/log.h>

timer_descriptor Ticker::timer_tick;

static std_msgs__msg__Int32 msg_tick;

static publisher_descriptor pdescriptor_tick;
static service_descriptor sdescriptor_addtwoints;

static example_interfaces__srv__AddTwoInts_Request req_addtwoints;
static example_interfaces__srv__AddTwoInts_Response res_addtwoints;

#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { return false; } \
  }
#define RCNOCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    (void)temp_rc; \
  }

Ticker::Ticker() {
  msg_tick.data = 0;
};

static void tick_cb(int64_t last_call_time) {
  //micro_rosso::set_timestamp(msg_tick.header.stamp);
  RCNOCHECK(rcl_publish(&pdescriptor_tick.publisher, &msg_tick, NULL));
  msg_tick.data++;
}

static void service_addtwoints_cb(const void* req, void* res) {
  /*
  example_interfaces__srv__Trigger_Response* res_in =  
    (example_interfaces__srv__Trigger_Response*)res;   
  res_in->success = true; // time_sync();
  */
  D_print("# ");

  example_interfaces__srv__AddTwoInts_Request* req_in =
    (example_interfaces__srv__AddTwoInts_Request*)req;
  example_interfaces__srv__AddTwoInts_Response* res_in =
    (example_interfaces__srv__AddTwoInts_Response*)res;

  D_print((int64_t)req_in->a);
  D_print("+");
  D_print((int64_t)req_in->b);
  D_println();

  res_in->sum = (int64_t)req_in->a + (int64_t)req_in->b;
  //res_in.success = true;
}

static void timer_handler_tick (rcl_timer_t* timer, int64_t last_call_time) {
  for (int j = 0; j < Ticker::timer_tick.callbacks.size(); j++) {
    Ticker::timer_tick.callbacks[j](last_call_time);
  }
  return;
}

bool Ticker::setup() {
  D_println("setup: ticker");

  pdescriptor_tick.qos = QOS_DEFAULT;
  pdescriptor_tick.type_support =
    (rosidl_message_type_support_t*)ROSIDL_GET_MSG_TYPE_SUPPORT(
      std_msgs, msg, Int32);
  pdescriptor_tick.topic_name = TICKER_TOPIC_TICK;
  micro_rosso::publishers.push_back(&pdescriptor_tick);

  sdescriptor_addtwoints.qos = QOS_DEFAULT;
  sdescriptor_addtwoints.type_support =
    ROSIDL_GET_SRV_TYPE_SUPPORT(example_interfaces, srv, AddTwoInts);
  sdescriptor_addtwoints.service_name = TICKER_SERVICE_ADD;
  sdescriptor_addtwoints.request = &req_addtwoints;
  sdescriptor_addtwoints.response = &res_addtwoints;
  sdescriptor_addtwoints.callback = service_addtwoints_cb;

  //do not enable unless reconfigured microros for more services
  //micro_rosso::services.push_back(&sdescriptor_addtwoints);

  // create a new timer
  Ticker::timer_tick.timeout_ns = RCL_MS_TO_NS(TIMER_TICK_MS);
  Ticker::timer_tick.timer_handler = timer_handler_tick;
  Ticker::timer_tick.callbacks.push_back(&tick_cb);
  micro_rosso::timers.push_back(&Ticker::timer_tick);

  return true;
}

#endif // BUILD_TICKER

