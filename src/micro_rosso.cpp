#include "micro_rosso.h"

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

#if ROS_PARAMETER_SERVER
rclc_parameter_server_t micro_rosso::param_server;
#endif

std::vector<publisher_descriptor *> micro_rosso::publishers;
std::vector<subscriber_descriptor *> micro_rosso::subscribers;
std::vector<service_descriptor *> micro_rosso::services;
std::vector<client_descriptor *> micro_rosso::clients;
std::vector<timer_descriptor *> micro_rosso::timers;
std::vector<void (*)(ros_states)> micro_rosso::ros_state_listeners;
std::vector<void (*)(void)> micro_rosso::post_init;
#if ROS_PARAMETER_SERVER
std::vector<void (*)(const Parameter *, const Parameter *)> micro_rosso::parameter_change_listeners;
#endif

timer_descriptor micro_rosso::timer_control;
timer_descriptor micro_rosso::timer_report;

static rclc_support_t support;
static rcl_node_t node;
static rcl_allocator_t allocator;
static rclc_executor_t executor;

static const char *ros2_node_name; // ROS2_NODE_NAME

ros_states ros_state;
ros_states ros_state_last;

/*
#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { return false; } \
  }
*/
#define RCCHECK(fn)                                       \
  {                                                       \
    rcl_ret_t temp_rc = fn;                               \
    if ((temp_rc != RCL_RET_OK))                          \
    {                                                     \
      D_print("micro_rosso.cpp failed on [line:code]: "); \
      D_print((int)__LINE__);                             \
      D_print(":");                                       \
      D_print((int)temp_rc);                              \
      D_println();                                        \
      rcutils_reset_error();                              \
      return 1;                                           \
    }                                                     \
  }
#define RCNOCHECK(fn)       \
  {                         \
    rcl_ret_t temp_rc = fn; \
    (void)temp_rc;          \
  }
#define EXECUTE_EVERY_N_MS(MS, X)      \
  do                                   \
  {                                    \
    static volatile int64_t init = -1; \
    if (init == -1)                    \
    {                                  \
      init = uxr_millis();             \
    }                                  \
    if (uxr_millis() - init > MS)      \
    {                                  \
      X;                               \
      init = uxr_millis();             \
    }                                  \
  } while (0)

micro_rosso::micro_rosso()
{
}

bool micro_rosso::time_sync()
{
  micro_rosso::logger.log("Time synching... ");

  RCCHECK(rmw_uros_sync_session(1000));

  int64_t time_ms = rmw_uros_epoch_millis();
  if (time_ms > 0)
  {
    char epoch_str[20 + 21]{0};
    sprintf(epoch_str, "uros_epoch_millis: %" PRIu64, time_ms);
    micro_rosso::logger.log(epoch_str);
  }
  else
  {
    char error_str[50];
    sprintf(error_str, "Session time sync failed, error code: %i", (int)time_ms);
    micro_rosso::logger.log(error_str, __FILE__, __func__, __LINE__, rcl_interfaces__msg__Log__ERROR);
    return false;
  }

  return true;
}

void micro_rosso::set_timestamp(builtin_interfaces__msg__Time &stamp)
{
  int64_t now = rmw_uros_epoch_millis();
  stamp.sec = now / 1000;
  stamp.nanosec = (now % 1000) * 1000000;
}

void micro_rosso::set_timestamp_ms(builtin_interfaces__msg__Time &stamp, int64_t now)
{
  stamp.sec = now / 1000;
  stamp.nanosec = (now % 1000) * 1000000;
}

void micro_rosso::set_timestamp_ns(builtin_interfaces__msg__Time &stamp, int64_t now)
{
  stamp.sec = now / 1000000000;
  stamp.nanosec = now % 1000000000;
}

static void shrink_to_fit()
{
  micro_rosso::publishers.shrink_to_fit();
  micro_rosso::subscribers.shrink_to_fit();
  micro_rosso::services.shrink_to_fit();
  micro_rosso::clients.shrink_to_fit();
  micro_rosso::timers.shrink_to_fit();
  micro_rosso::ros_state_listeners.shrink_to_fit();
  micro_rosso::post_init.shrink_to_fit();
  for (int i = 0; i < micro_rosso::timers.size(); i++)
  {
    timer_descriptor *t = micro_rosso::timers[i];
    t->callbacks.shrink_to_fit();
  }
#if ROS_PARAMETER_SERVER
  micro_rosso::parameter_change_listeners.shrink_to_fit();
#endif
}

static void timer_handler_control(rcl_timer_t *timer, int64_t last_call_time)
{
  for (int j = 0; j < micro_rosso::timer_control.callbacks.size(); j++)
  {
    micro_rosso::timer_control.callbacks[j](last_call_time);
  }
  return;
}

static void timer_handler_report(rcl_timer_t *timer, int64_t last_call_time)
{
  for (int j = 0; j < micro_rosso::timer_report.callbacks.size(); j++)
  {
    micro_rosso::timer_report.callbacks[j](last_call_time);
  }
  return;
}

#if ROS_PARAMETER_SERVER
static void print_parameter_desc(const Parameter *p)
{
  if (p == NULL)
  {
    D_print("NULL");
  }
  else
  {
    D_print(p->name.data);
    switch (p->value.type)
    {
    case RCLC_PARAMETER_BOOL:
      D_print("=(bool)");
      D_print(p->value.bool_value);
      break;
    case RCLC_PARAMETER_INT:
      D_print("=(int)");
      D_print(p->value.integer_value);
      break;
    case RCLC_PARAMETER_DOUBLE:
      D_print("=(double)");
      D_print(p->value.double_value);
      break;
    default:
      break;
    }
  }
}

bool on_parameter_changed(const Parameter *old_param, const Parameter *new_param, void *context)
{
  (void)context;
  if (old_param == NULL && new_param == NULL)
  {
    D_println("ERROR: on_parameter_changed, both parameters are NULL");
    return false;
  }
  D_print("parameter change: ");
  print_parameter_desc(old_param);
  D_print(" -> ");
  print_parameter_desc(new_param);
  D_println();

  for (int i = 0; i < micro_rosso::parameter_change_listeners.size(); i++)
  {
    micro_rosso::parameter_change_listeners[i](old_param, new_param);
  }
  return true;
}
#endif

bool micro_rosso::setup(const char *rosname)
{
  
#if defined(DEBUG_CONSOLE)
#if !defined(DEBUG_CONSOLE_PIN_RX)
#define DEBUG_CONSOLE_PIN_RX -1
#endif
#if !defined(DEBUG_CONSOLE_PIN_TX)
#define DEBUG_CONSOLE_PIN_TX -1
#endif
D_SerialBegin(DEBUG_CONSOLE_BAUD, SERIAL_8N1, DEBUG_CONSOLE_PIN_RX, DEBUG_CONSOLE_PIN_TX);
#endif

  D_println("Setting up micro_rosso started... ");

  reset_reason_0 = rtc_get_reset_reason(0);
  reset_reason_1 = rtc_get_reset_reason(1);

  ros2_node_name = rosname;

  // delay(3000);

  ros_state = WAITING_AGENT;
  ros_state_last = AGENT_DISCONNECTED; // trigger first notification

  if (!micro_rosso::logger.setup())
  {
    D_println("FAIL logger.setup()");
    return false;
  };

  micro_rosso::timer_control.timeout_ns = RCL_MS_TO_NS(TIMER_CONTROL_MS);
  micro_rosso::timer_control.timer_handler = timer_handler_control;
  micro_rosso::timers.push_back(&micro_rosso::timer_control);

  micro_rosso::timer_report.timeout_ns = RCL_MS_TO_NS(TIMER_REPORT_MS);
  micro_rosso::timer_report.timer_handler = timer_handler_report;
  micro_rosso::timers.push_back(&micro_rosso::timer_report);

  D_println("Setting up micro_rosso done.");
  return true;
}

static const char *reset_reason_string(const RESET_REASON reason)
{
  switch (reason)
  {
  case 1:
    return "POWERON_RESET"; /**<1, Vbat power on reset*/
  case 3:
    return "SW_RESET"; /**<3, Software reset digital core*/
  case 4:
    return "OWDT_RESET"; /**<4, Legacy watch dog reset digital core*/
  case 5:
    return "DEEPSLEEP_RESET"; /**<5, Deep Sleep reset digital core*/
  case 6:
    return "SDIO_RESET"; /**<6, Reset by SLC module, reset digital core*/
  case 7:
    return "TG0WDT_SYS_RESET"; /**<7, Timer Group0 Watch dog reset digital core*/
  case 8:
    return "TG1WDT_SYS_RESET"; /**<8, Timer Group1 Watch dog reset digital core*/
  case 9:
    return "RTCWDT_SYS_RESET"; /**<9, RTC Watch dog Reset digital core*/
  case 10:
    return "INTRUSION_RESET"; /**<10, Instrusion tested to reset CPU*/
  case 11:
    return "TGWDT_CPU_RESET"; /**<11, Time Group reset CPU*/
  case 12:
    return "SW_CPU_RESET"; /**<12, Software reset CPU*/
  case 13:
    return "RTCWDT_CPU_RESET"; /**<13, RTC Watch dog Reset CPU*/
  case 14:
    return "EXT_CPU_RESET"; /**<14, for APP CPU, reseted by PRO CPU*/
  case 15:
    return "RTCWDT_BROWN_OUT_RESET"; /**<15, Reset when the vdd voltage is not stable*/
  case 16:
    return "RTCWDT_RTC_RESET"; /**<16, RTC Watch dog reset digital core and rtc module*/
  default:
    return "";
  }
}

static const char *ros_state_string(const ros_states state)
{
  switch (state)
  {
  case WAITING_AGENT:
    return "WAITING_AGENT";
  case AGENT_AVAILABLE:
    return "AGENT_AVAILABLE";
  case AGENT_CONNECTED:
    return "AGENT_CONNECTED";
  case AGENT_DISCONNECTED:
    return "AGENT_DISCONNECTED";
  default:
    return "";
  }
};

// Functions create_entities and destroy_entities can take several seconds.
// In order to reduce this rebuild the library with
// - RMW_UXRCE_ENTITY_CREATION_DESTROY_TIMEOUT=0
// - UCLIENT_MAX_SESSION_CONNECTION_ATTEMPTS=3
static bool create_entities()
{
  D_print("Creating ROS2 entities for node: ");
  D_println(ros2_node_name);
  shrink_to_fit();

  D_print("publishers: ");
  D_print(micro_rosso::publishers.size());
  D_print(",  RMW_UXRCE_MAX_PUBLISHERS: ");
  D_println(RMW_UXRCE_MAX_PUBLISHERS);

  D_print("subscriptions: ");
  D_print(micro_rosso::subscribers.size());
  D_print(", RMW_UXRCE_MAX_SUBSCRIPTIONS: ");
  D_println(RMW_UXRCE_MAX_SUBSCRIPTIONS);

  D_print("clients: ");
  D_print(micro_rosso::clients.size());
  D_print(", RMW_UXRCE_MAX_CLIENTS: ");
  D_println(RMW_UXRCE_MAX_CLIENTS);

  D_print("services: ");
  D_print(micro_rosso::services.size());
  D_print(", RMW_UXRCE_MAX_SERVICES: ");
  D_println(RMW_UXRCE_MAX_SERVICES);

#if ROS_PARAMETER_SERVER
  D_print("parameter server services (add to 'services' above): ");
  D_println(RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES);
  D_print("parameter change listeners: ");
  D_println(micro_rosso::parameter_change_listeners.size());
#endif
  D_print("state listeners: ");
  D_println(micro_rosso::ros_state_listeners.size());
  D_print("post init callbacks: ");
  D_println(micro_rosso::post_init.size());

  allocator = rcl_get_default_allocator();

  // create init_options
  //RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));

  // Set ROS domain id
  #if defined(ROS_DOMAIN_ID)
  D_print("Setting ROS_DOMAIN_ID: ");
  D_println(ROS_DOMAIN_ID);
  RCCHECK(rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID));
  #endif

  // Setup support structure.
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, ros2_node_name, "", &support));

  micro_rosso::time_sync();

  // create publisher
  for (int i = 0; i < micro_rosso::publishers.size(); i++)
  {
    publisher_descriptor *p = micro_rosso::publishers[i];
    D_print("+publisher: ");
    D_println(p->topic_name);

    switch (p->qos)
    {
    case QOS_DEFAULT:
      RCCHECK(rclc_publisher_init_default(
          &(p->publisher), &node, p->type_support, p->topic_name));
      break;
    case QOS_BEST_EFFORT:
      RCCHECK(rclc_publisher_init_best_effort(
          &(p->publisher), &node, p->type_support, p->topic_name));
      break;
    case QOS_CUSTOM:
      RCCHECK(rclc_publisher_init(
          &(p->publisher), &node, p->type_support, p->topic_name, p->qos_profile));
      break;
    }
  }

  // create subscriber
  for (int i = 0; i < micro_rosso::subscribers.size(); i++)
  {
    subscriber_descriptor *s = micro_rosso::subscribers[i];
    D_print("+subscriber: ");
    D_println(s->topic_name);

    switch (s->qos)
    {
    case QOS_DEFAULT:
      RCCHECK(rclc_subscription_init_default(
          &(s->subscriber), &node, s->type_support, s->topic_name));
      break;
    case QOS_BEST_EFFORT:
      RCCHECK(rclc_subscription_init_best_effort(
          &(s->subscriber), &node, s->type_support, s->topic_name));
      break;
    case QOS_CUSTOM:
      RCCHECK(rclc_subscription_init(
          &(s->subscriber), &node, s->type_support, s->topic_name, s->qos_profile));
      break;
    }
  }

  // create timers
  for (int i = 0; i < micro_rosso::timers.size(); i++)
  {
    timer_descriptor *t = micro_rosso::timers[i];
    D_print("+timer (ms): ");
    D_println(t->timeout_ns / 1000000);
    // deprecated:
    // RCCHECK(rclc_timer_init_default(
    //     &(t->timer), &support, t->timeout_ns, t->timer_handler));
    RCCHECK(rclc_timer_init_default2(
        &(t->timer), &support, t->timeout_ns, t->timer_handler, true));
  }

  // create services
  for (int i = 0; i < micro_rosso::services.size(); i++)
  {
    service_descriptor *s = micro_rosso::services[i];
    D_print("+service: ");
    D_println(s->service_name);

    switch (s->qos)
    {
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

  // create service clients
  for (int i = 0; i < micro_rosso::clients.size(); i++)
  {
    client_descriptor *c = micro_rosso::clients[i];
    D_print("+client: ");
    D_println(c->service_name);

    switch (c->qos)
    {
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

#if ROS_PARAMETER_SERVER
  // create service for parameter service
  D_println("+parameter server");
  RCCHECK(rclc_parameter_server_init_with_option(
      &micro_rosso::param_server, &node, &parameter_options));
  // RCCHECK(rclc_parameter_server_init_default(&param_server, &node));
#endif

  // create executor
  executor = rclc_executor_get_zero_initialized_executor();
  size_t n_executors =
      micro_rosso::timers.size() +
      micro_rosso::services.size() +
      micro_rosso::clients.size() +
      micro_rosso::subscribers.size();

#if ROS_PARAMETER_SERVER
  n_executors += RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES;
  // RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES ?
#endif

  D_print("Allocating executors: ");
  D_println(n_executors);

  RCCHECK(rclc_executor_init(&executor, &support.context, n_executors, &allocator));
  for (int i = 0; i < micro_rosso::timers.size(); i++)
  {
    timer_descriptor *t = micro_rosso::timers[i];
    RCCHECK(rclc_executor_add_timer(&executor, &(t->timer)));
  }

  for (int i = 0; i < micro_rosso::subscribers.size(); i++)
  {
    subscriber_descriptor *s = micro_rosso::subscribers[i];
    RCCHECK(rclc_executor_add_subscription(
        &executor, &(s->subscriber), s->msg, s->callback, ON_NEW_DATA));
  }

  for (int i = 0; i < micro_rosso::services.size(); i++)
  {
    service_descriptor *s = micro_rosso::services[i];
    RCCHECK(rclc_executor_add_service(
        &executor, &(s->service), s->request, s->response, s->callback));
  }

  for (int i = 0; i < micro_rosso::clients.size(); i++)
  {
    client_descriptor *c = micro_rosso::clients[i];
    RCCHECK(rclc_executor_add_client(
        &executor, &(c->client), c->response, c->callback));
  }

#if ROS_PARAMETER_SERVER
  rclc_executor_add_parameter_server(
      &executor, &micro_rosso::param_server, on_parameter_changed);
#endif

  D_println("Creation of ROS2 entities finished.");
  delay(500);

  micro_rosso::logger.log("ROS2 Connected.");

  if (reset_reason_0 > 0)
  {
    char reset_reason_str[50]{0};
    sprintf(reset_reason_str, "Coming from Reset Core0: %d %s", reset_reason_0, reset_reason_string(reset_reason_0));
    micro_rosso::logger.log(reset_reason_str);
    reset_reason_0 = (RESET_REASON)0;
  }
  if (reset_reason_1 > 0)
  {
    char reset_reason_str[50]{0};
    sprintf(reset_reason_str, "Coming from Reset Core1: %d %s", reset_reason_1, reset_reason_string(reset_reason_1));
    micro_rosso::logger.log(reset_reason_str);
    reset_reason_1 = (RESET_REASON)0;
  }

  for (int i = 0; i < micro_rosso::post_init.size(); i++)
  {
    micro_rosso::post_init[i]();
  }

  return true;
}

static void destroy_entities()
{
  D_print("Destroying ROS2 entities... ");

  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  for (int i = 0; i < micro_rosso::publishers.size(); i++)
  {
    publisher_descriptor *p = micro_rosso::publishers[i];
    // D_print("-publisher: "); D_println(p->topic_name);
    RCNOCHECK(rcl_publisher_fini(&(p->publisher), &node));
  }
  for (int i = 0; i < micro_rosso::subscribers.size(); i++)
  {
    subscriber_descriptor *s = micro_rosso::subscribers[i];
    // D_print("-subscriber: "); D_println(s->topic_name);
    RCNOCHECK(rcl_subscription_fini(&(s->subscriber), &node));
  }
  for (int i = 0; i < micro_rosso::services.size(); i++)
  {
    service_descriptor *s = micro_rosso::services[i];
    // D_print("-service: "); D_println(s->service_name);
    RCNOCHECK(rcl_service_fini(&(s->service), &node));
  }
  for (int i = 0; i < micro_rosso::clients.size(); i++)
  {
    client_descriptor *c = micro_rosso::clients[i];
    // D_print("-service: "); D_println(s->service_name);
    RCNOCHECK(rcl_client_fini(&(c->client), &node));
  }
  for (int i = 0; i < micro_rosso::timers.size(); i++)
  {
    timer_descriptor *t = micro_rosso::timers[i];
    // D_print("-timer (ms): ");D_println(t->timeout_ns / 1000000);
    RCNOCHECK(rcl_timer_fini(&(t->timer)));
  }

  RCNOCHECK(rclc_executor_fini(&executor));
  RCNOCHECK(rcl_node_fini(&node));
  RCNOCHECK(rclc_support_fini(&support));

#if ROS_PARAMETER_SERVER
  RCNOCHECK(rclc_parameter_server_fini(&micro_rosso::param_server, &node););
#endif

  D_println("done.");
}

void micro_rosso::loop()
{
  switch (ros_state)
  {
  case WAITING_AGENT:
    EXECUTE_EVERY_N_MS(500, ros_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
    break;
  case AGENT_AVAILABLE:
    ros_state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
    if (ros_state == WAITING_AGENT)
    {
      destroy_entities();
    };
    break;
  case AGENT_CONNECTED:
    EXECUTE_EVERY_N_MS(200, ros_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
    if (ros_state == AGENT_CONNECTED)
    {
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

  if (ros_state != ros_state_last)
  {
    D_print("ROS2 state: ");
    D_println(ros_state_string(ros_state));

    for (int i = 0; i < micro_rosso::ros_state_listeners.size(); i++)
    {
      micro_rosso::ros_state_listeners[i](ros_state);
    }
    ros_state_last = ros_state;
  }
}
