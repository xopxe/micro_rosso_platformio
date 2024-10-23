#include "micro_rosso.h"

#ifdef USE_SET_TIME
#include <TimeLib32.h>
#endif

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rom/rtc.h>

#if defined(TRANSPORT_WIFI_STATIC_IP)
#include <WiFi.h>
#endif

static RESET_REASON reset_reason_0;
static RESET_REASON reset_reason_1;

std::vector<publisher_descriptor*> micro_rosso::publishers;
std::vector<subscriber_descriptor*> micro_rosso::subscribers;
std::vector<service_descriptor*> micro_rosso::services;
std::vector<client_descriptor*> micro_rosso::clients;
std::vector<timer_descriptor*> micro_rosso::timers;
std::vector<void (*) (ros_states)> micro_rosso::ros_state_listeners;

timer_descriptor micro_rosso::timer_control;
timer_descriptor micro_rosso::timer_report;

static rclc_support_t support;
static rcl_node_t node;
static rcl_allocator_t allocator;
static rclc_executor_t executor;

ros_states ros_state;
ros_states ros_state_last;

static unsigned long long ros_time_offset_ms;

/*
#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { return false; } \
  }
*/
#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { \
      D_print("micro_rosso failed on [line:code]: "); \
      D_print((int)__LINE__); \
      D_print(" "); \
      D_print((int)temp_rc); \
      D_println(); \
      rcutils_reset_error(); \
      return 1; \
    } \
  }
#define RCNOCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    (void)temp_rc; \
  }
#define EXECUTE_EVERY_N_MS(MS, X) \
  do { \
    static volatile int64_t init = -1; \
    if (init == -1) { init = uxr_millis(); } \
    if (uxr_millis() - init > MS) { \
      X; \
      init = uxr_millis(); \
    } \
  } while (0)

micro_rosso::micro_rosso() {
}

bool micro_rosso::time_sync() {
  micro_rosso::logger.log("Time synching... ");

  RCCHECK(rmw_uros_sync_session(1000));

  int64_t time_ms = rmw_uros_epoch_millis();
  if (time_ms > 0) {
    ros_time_offset_ms = time_ms - millis();
    char offset_str[20 + 21] {0};
    //memset(offset_str, 0x00, 20 + 11);
    sprintf(offset_str, "Agent time offset: %" PRIu64, ros_time_offset_ms);
    micro_rosso::logger.log(offset_str);
#ifdef USE_SET_TIME
    time_t time_seconds = time_ms / 1000;
    char time_str[12 + 25] {0};
    //memset(time_str, 0x00, 12 + 25);
    setTime(time_seconds);
    sprintf(time_str, "Agent date: %02d.%02d.%04d %02d:%02d:%02d.%03d", 
      day(), month(), year(), hour(), minute(), second(), (uint)time_ms % 1000);
    micro_rosso::logger.log(time_str);
#endif
  } else {
    char error_str[50];
    sprintf(error_str, "Session time sync failed, error code: %i", (int)time_ms);
    micro_rosso::logger.log(error_str, __FILE__, __func__, __LINE__, rcl_interfaces__msg__Log__ERROR);
    return false;
  }

  return true;
}

void micro_rosso::set_timestamp(builtin_interfaces__msg__Time& stamp) {
  unsigned long now = millis() + ros_time_offset_ms;
  stamp.sec = now / 1000;
  stamp.nanosec = (now % 1000) * 1000000;
}

static void shrink_to_fit() {
  micro_rosso::publishers.shrink_to_fit();
  micro_rosso::subscribers.shrink_to_fit();
  micro_rosso::services.shrink_to_fit();
  micro_rosso::clients.shrink_to_fit();
  micro_rosso::timers.shrink_to_fit();
  micro_rosso::ros_state_listeners.shrink_to_fit();
  for (int i = 0; i < micro_rosso::timers.size(); i++) {
    timer_descriptor* t = micro_rosso::timers[i];
    t->callbacks.shrink_to_fit();
  }
}

static void timer_handler_control (rcl_timer_t* timer, int64_t last_call_time) {
  for (int j = 0; j < micro_rosso::timer_control.callbacks.size(); j++) {
    micro_rosso::timer_control.callbacks[j](last_call_time);
  }
  return;
}

static void timer_handler_report (rcl_timer_t* timer, int64_t last_call_time) {
  for (int j = 0; j < micro_rosso::timer_report.callbacks.size(); j++) {
    micro_rosso::timer_report.callbacks[j](last_call_time);
  }
  return;
}


bool micro_rosso::setup() {
  reset_reason_0 = rtc_get_reset_reason(0);
  reset_reason_1 = rtc_get_reset_reason(1);

  //delay(3000);

  ros_state = WAITING_AGENT;
  ros_state_last = AGENT_DISCONNECTED; // trigger first notification

  D_println("Setting up transport... ");
  #if defined(TRANSPORT_WIFI)
  #if defined(TRANSPORT_WIFI_STATIC_IP)
  WiFi.config(TRANSPORT_WIFI_STATIC_IP,TRANSPORT_WIFI_STATIC_GATEWAY, TRANSPORT_WIFI_STATIC_SUBNET);
  #endif
  set_microros_wifi_transports(
    TRANSPORT_WIFI_SSID, 
    TRANSPORT_WIFI_PASS, 
    TRANSPORT_WIFI_AGENT_IP, 
    TRANSPORT_WIFI_PORT);
  #elif defined(TRANSPORT_SERIAL)
  Serial.begin(TRANSPORT_SERIAL_BAUD);
  set_microros_serial_transports(TRANSPORT_SERIAL);
  #else
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  #endif
  D_println("Done.");

  if (!micro_rosso::logger.setup()) {
    D_println("FAIL logger.setup()");
  };

  Wire.begin(I2C_SDA, I2C_SCL);

  micro_rosso::timer_control.timeout_ns = RCL_MS_TO_NS(TIMER_CONTROL_MS);
  micro_rosso::timer_control.timer_handler = timer_handler_control;
  micro_rosso::timers.push_back(&micro_rosso::timer_control);

  micro_rosso::timer_report.timeout_ns = RCL_MS_TO_NS(TIMER_REPORT_MS);
  micro_rosso::timer_report.timer_handler = timer_handler_report;
  micro_rosso::timers.push_back(&micro_rosso::timer_report);
  
  return true;
}

static const char* reset_reason_string(const RESET_REASON reason)
{
  switch (reason)
  {
    case 1 : return "POWERON_RESET";          /**<1, Vbat power on reset*/
    case 3 : return "SW_RESET";               /**<3, Software reset digital core*/
    case 4 : return "OWDT_RESET";             /**<4, Legacy watch dog reset digital core*/
    case 5 : return "DEEPSLEEP_RESET";        /**<5, Deep Sleep reset digital core*/
    case 6 : return "SDIO_RESET";             /**<6, Reset by SLC module, reset digital core*/
    case 7 : return "TG0WDT_SYS_RESET";       /**<7, Timer Group0 Watch dog reset digital core*/
    case 8 : return "TG1WDT_SYS_RESET";       /**<8, Timer Group1 Watch dog reset digital core*/
    case 9 : return "RTCWDT_SYS_RESET";       /**<9, RTC Watch dog Reset digital core*/
    case 10 : return "INTRUSION_RESET";       /**<10, Instrusion tested to reset CPU*/
    case 11 : return "TGWDT_CPU_RESET";       /**<11, Time Group reset CPU*/
    case 12 : return "SW_CPU_RESET";          /**<12, Software reset CPU*/
    case 13 : return "RTCWDT_CPU_RESET";      /**<13, RTC Watch dog Reset CPU*/
    case 14 : return "EXT_CPU_RESET";         /**<14, for APP CPU, reseted by PRO CPU*/
    case 15 : return "RTCWDT_BROWN_OUT_RESET";/**<15, Reset when the vdd voltage is not stable*/
    case 16 : return "RTCWDT_RTC_RESET";      /**<16, RTC Watch dog reset digital core and rtc module*/
    default : return "";
  }
}

// Functions create_entities and destroy_entities can take several seconds.
// In order to reduce this rebuild the library with
// - RMW_UXRCE_ENTITY_CREATION_DESTROY_TIMEOUT=0
// - UCLIENT_MAX_SESSION_CONNECTION_ATTEMPTS=3
static bool create_entities() {
  D_println("Creating ROS2 entities... ");

  shrink_to_fit();
  D_print("publishers: ");
  D_println(micro_rosso::publishers.size());
  D_print("subscriptions: ");
  D_println(micro_rosso::subscribers.size());
  D_print("services: ");
  D_println(micro_rosso::services.size());
  D_print("clients: ");
  D_println(micro_rosso::clients.size());
  D_print("state listeners: ");
  D_println(micro_rosso::ros_state_listeners.size());

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, ROS2_NODE_NAME, "", &support));

  // create publisher
  for (int i = 0; i < micro_rosso::publishers.size(); i++) {
    publisher_descriptor* p = micro_rosso::publishers[i];
    D_print("+publisher: ");
    D_println(p->topic_name);
    
    switch (p->qos) {
      case QOS_DEFAULT:
        RCCHECK(rclc_publisher_init_default(
          &(p->publisher), &node, p->type_support, p->topic_name));
        break;
      case QOS_BEST_EFFORT:
        RCCHECK(rclc_publisher_init_best_effort(
          &(p->publisher), &node, p->type_support, p->topic_name));
        break;
    }
  }

  // create subscriber
  for (int i = 0; i < micro_rosso::subscribers.size(); i++) {
    subscriber_descriptor* s = micro_rosso::subscribers[i];
    D_print("+subscriber: ");
    D_println(s->topic_name);
    RCCHECK(rclc_subscription_init_default(
      &(s->subscriber), &node, s->type_support, s->topic_name));
  }

  // create timers,
  for (int i = 0; i < micro_rosso::timers.size(); i++) {
    timer_descriptor* t = micro_rosso::timers[i];
    D_print("+timer (ms): ");
    D_println(t->timeout_ns / 1000000);
    RCCHECK(rclc_timer_init_default(
      &(t->timer), &support, t->timeout_ns, t->timer_handler));
  }

  // create services
  if (micro_rosso::services.size() > 1) {
    D_println("WARNING: verify DRMW_UXRCE_MAX_SERVICES in colcon.meta ");
  }
  for (int i = 0; i < micro_rosso::services.size(); i++) {
    service_descriptor* s = micro_rosso::services[i];
    D_print("+service: ");
    D_println(s->service_name);
    
    switch (s->qos) {
      case QOS_DEFAULT:
        RCCHECK(rclc_service_init_default(
          &(s->service), &node, s->type_support, s->service_name));
        break;
      case QOS_BEST_EFFORT:
        RCCHECK(rclc_service_init_best_effort(
          &(s->service), &node, s->type_support, s->service_name));
        break;
    }
  }
  
  // create servce clients
  for (int i = 0; i < micro_rosso::clients.size(); i++) {
    client_descriptor* c = micro_rosso::clients[i];
    D_print("+client: ");
    D_println(c->service_name);
    
    switch (c->qos) {
      case QOS_DEFAULT:
        RCCHECK(rclc_client_init_default(
          &(c->client), &node, c->type_support, c->service_name));
        break;
      case QOS_BEST_EFFORT:
        RCCHECK(rclc_client_init_best_effort(
          &(c->client), &node, c->type_support, c->service_name));
        break;
    }
  }


  // create executor
  executor = rclc_executor_get_zero_initialized_executor();
  size_t n_executors =
    micro_rosso::timers.size() + 
    micro_rosso::services.size() + 
    micro_rosso::clients.size() + 
    micro_rosso::subscribers.size();

  D_print("Allocating executors: ");
  D_println(n_executors);

  RCCHECK(rclc_executor_init(&executor, &support.context, n_executors, &allocator));
  for (int i = 0; i < micro_rosso::timers.size(); i++) {
    timer_descriptor* t = micro_rosso::timers[i];
    RCCHECK(rclc_executor_add_timer(&executor, &(t->timer)));
  }

  for (int i = 0; i < micro_rosso::subscribers.size(); i++) {
    subscriber_descriptor* s = micro_rosso::subscribers[i];
    RCCHECK(rclc_executor_add_subscription(
      &executor, &(s->subscriber), s->msg, s->callback, ON_NEW_DATA));
  }

  for (int i = 0; i < micro_rosso::services.size(); i++) {
    service_descriptor* s = micro_rosso::services[i];
    RCCHECK(rclc_executor_add_service(
      &executor, &(s->service), s->request, s->response, s->callback));
  }

  for (int i = 0; i < micro_rosso::clients.size(); i++) {
    client_descriptor* c = micro_rosso::clients[i];
    RCCHECK(rclc_executor_add_client(
      &executor, &(c->client), c->response, c->callback));
  }
  
  D_println("...Done.");
  delay(500);

  micro_rosso::logger.log("ROS2 Connected.");
  
  if (reset_reason_0>0) {
    char reset_reason_str[50] {0};
    sprintf(reset_reason_str, "Coming from Reset Core0: %d %s", reset_reason_0, reset_reason_string(reset_reason_0));
    micro_rosso::logger.log(reset_reason_str);
    reset_reason_0 = (RESET_REASON)0;
  }
  if (reset_reason_1>0) {
    char reset_reason_str[50] {0};
    sprintf(reset_reason_str, "Coming from Reset Core1: %d %s", reset_reason_1, reset_reason_string(reset_reason_1));
    micro_rosso::logger.log(reset_reason_str);
    reset_reason_1 = (RESET_REASON)0;
  }

  micro_rosso::time_sync();

  return true;
}

static void destroy_entities() {
  D_println("Destroying ROS2 entities... ");

  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  for (int i = 0; i < micro_rosso::publishers.size(); i++) {
    publisher_descriptor* p = micro_rosso::publishers[i];
    //D_print("-publisher: "); D_println(p->topic_name);
    RCNOCHECK(rcl_publisher_fini(&(p->publisher), &node));
  }
  for (int i = 0; i < micro_rosso::subscribers.size(); i++) {
    subscriber_descriptor* s = micro_rosso::subscribers[i];
    //D_print("-subscriber: "); D_println(s->topic_name);
    RCNOCHECK(rcl_subscription_fini(&(s->subscriber), &node));
  }
  for (int i = 0; i < micro_rosso::services.size(); i++) {
    service_descriptor* s = micro_rosso::services[i];
    //D_print("-service: "); D_println(s->service_name);
    RCNOCHECK(rcl_service_fini(&(s->service), &node));
  }
  for (int i = 0; i < micro_rosso::clients.size(); i++) {
    client_descriptor* c = micro_rosso::clients[i];
    //D_print("-service: "); D_println(s->service_name);
    RCNOCHECK(rcl_client_fini(&(c->client), &node));
  }
  for (int i = 0; i < micro_rosso::timers.size(); i++) {
    timer_descriptor* t = micro_rosso::timers[i];
    //D_print("-timer (ms): ");D_println(t->timeout_ns / 1000000);
    RCNOCHECK(rcl_timer_fini(&(t->timer)));
  }


  RCNOCHECK(rclc_executor_fini(&executor));
  RCNOCHECK(rcl_node_fini(&node));
  RCNOCHECK(rclc_support_fini(&support));

  D_println("...Done.");
}

void micro_rosso::loop() {
  switch (ros_state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, ros_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      ros_state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (ros_state == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, ros_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (ros_state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      }
      break;
    case AGENT_DISCONNECTED:
      D_println("ROS2 Disconnnected.");
      destroy_entities();
      ros_state = WAITING_AGENT;
      break;
    default:
      break;
  }

  if (ros_state != ros_state_last) {
    D_print("ROS2 state: ");
    D_println(ros_state);

    for (int i = 0; i < micro_rosso::ros_state_listeners.size(); i++) {
      micro_rosso::ros_state_listeners[i] (ros_state);
    }
    ros_state_last = ros_state;
  }  
}

