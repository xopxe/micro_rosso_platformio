#ifndef __micro_rosso_h
#define __micro_rosso_h

#include <Arduino.h>
#include <micro_ros_platformio.h>

#include "micro_rosso_config.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <vector>

enum QosMode
{
  QOS_DEFAULT,
  QOS_BEST_EFFORT,
  QOS_CUSTOM
};

enum ros_states
{
  WAITING_AGENT,     // the client is waiting for an agent
  AGENT_AVAILABLE,   // agent detected, connecting
  AGENT_CONNECTED,   // client connected to agent
  AGENT_DISCONNECTED // connection to agent broken, disconnecting
};

#if defined(DEBUG_CONSOLE)
#define D_SerialBegin(...) DEBUG_CONSOLE.begin(__VA_ARGS__);
#define D_print(...) DEBUG_CONSOLE.print(__VA_ARGS__)
#define D_write(...) DEBUG_CONSOLE.write(__VA_ARGS__)
#define D_println(...) DEBUG_CONSOLE.println(__VA_ARGS__)
#else
#define D_SerialBegin(...)
#define D_print(...)
#define D_write(...)
#define D_println(...)
#endif

#include "logger.h"

struct publisher_descriptor
{
  QosMode qos;
  rmw_qos_profile_t *qos_profile; // when qos is QOS_CUSTOM
  rcl_publisher_t publisher;
  const rosidl_message_type_support_t *type_support;
  const char *topic_name;
};

struct subscriber_descriptor
{
  rcl_subscription_t subscriber;
  const rosidl_message_type_support_t *type_support;
  const char *topic_name;
  void *msg;
  rclc_subscription_callback_t callback;
};

struct service_descriptor
{
  QosMode qos;
  rcl_service_t service;
  const rosidl_service_type_support_t *type_support;
  const char *service_name;
  void *request;
  void *response;
  rclc_service_callback_t callback;
};

struct client_descriptor
{
  QosMode qos;
  rcl_client_t client;
  const rosidl_service_type_support_t *type_support;
  const char *service_name;
  void *request;
  void *response;
  rclc_client_callback_t callback;
};

struct timer_descriptor
{
  rcl_timer_t timer;
  uint64_t timeout_ns;
  rcl_timer_callback_t timer_handler;
  std::vector<void (*)(int64_t last_call_time)> callbacks;
};

class micro_rosso
{

public:
  static bool setup(const char *ros2_node_name);

  static Logger logger;
  #if ROS_PARAMETER_SERVER
  static rclc_parameter_server_t param_server;
  #endif

  static std::vector<publisher_descriptor *> publishers;
  static std::vector<subscriber_descriptor *> subscribers;
  static std::vector<service_descriptor *> services;
  static std::vector<client_descriptor *> clients;
  static std::vector<timer_descriptor *> timers;
  static std::vector<void (*)(ros_states)> ros_state_listeners;
  static std::vector<void (*)(void)> post_init;
  #if ROS_PARAMETER_SERVER
  static std::vector<void (*) (const Parameter*, const Parameter*)>parameter_change_listeners;
  #endif

  static timer_descriptor timer_control;
  static timer_descriptor timer_report;

  static void set_timestamp(builtin_interfaces__msg__Time &stamp);
  static void set_timestamp_ms(builtin_interfaces__msg__Time &stamp, int64_t now);
  static void set_timestamp_ns(builtin_interfaces__msg__Time &stamp, int64_t now);

  static bool time_sync();

  static void loop();

  micro_rosso();
  micro_rosso(micro_rosso const &);    // Don't Implement
  void operator=(micro_rosso const &); // Don't implement
};

#endif // __micro_rosso_h
