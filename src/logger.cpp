#include "micro_rosso.h"

#include "logger.h"

#include <rcl_interfaces/msg/log.h>

rcl_interfaces__msg__Log msg_log;

static publisher_descriptor pdescriptor_roslog;


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


Logger::Logger() {}

bool Logger::setup() {
  D_println("setup: roslogger");

  msg_log.name.data = (char*)LOGGER_TOPIC_LOG;
  msg_log.name.size = strlen(msg_log.name.data);

  pdescriptor_roslog.qos = QOS_DEFAULT;
  pdescriptor_roslog.type_support = (rosidl_message_type_support_t*)
    ROSIDL_GET_MSG_TYPE_SUPPORT(rcl_interfaces, msg, Log);
  pdescriptor_roslog.topic_name = LOGGER_TOPIC_LOG;
  micro_rosso::publishers.push_back(&pdescriptor_roslog);

  return true;
}


void Logger::log(const char* s,
                 const char* file,
                 const char* func,
                 uint32_t line,
                 uint8_t level) {
  micro_rosso::set_timestamp(msg_log.stamp);
  msg_log.level = level;
  msg_log.msg.data = (char*)s;
  msg_log.msg.size = strlen(msg_log.msg.data);
  msg_log.file.data = (char*)file;
  msg_log.file.size = strlen(msg_log.file.data);
  msg_log.function.data = (char*)func;
  msg_log.function.size = strlen(msg_log.function.data);
  msg_log.line = line;

  D_println(s);

  RCNOCHECK(rcl_publish(&pdescriptor_roslog.publisher, &msg_log, NULL));
}
