#include "build_modules.h"
#ifdef BUILD_SYNC_TIME

#include "micro_rosso.h"

#include "sync_time.h"
#include <example_interfaces/srv/trigger.h>


static service_descriptor sdescriptor_sync_time;

example_interfaces__srv__Trigger_Request req_sync_time;
example_interfaces__srv__Trigger_Response res_sync_time;
static const char res_sync_time_message = '\0';  // https://github.com/micro-ROS/micro_ros_arduino/issues/101

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

SyncTime::SyncTime(){};

static void service_sync_time_cb(const void* req, void* res) {
  //example_interfaces__srv__Trigger_Response* res_in = (example_interfaces__srv__Trigger_Response*)res;

  res_sync_time.success = micro_rosso::time_sync();
}


bool SyncTime::setup() {
  D_println("init time syncer");

  res_sync_time.message.data = (char*)&res_sync_time_message;
  res_sync_time.message.size = 1;
  res_sync_time.message.capacity = 1;

  sdescriptor_sync_time.qos = QOS_DEFAULT;
  sdescriptor_sync_time.type_support =
    ROSIDL_GET_SRV_TYPE_SUPPORT(example_interfaces, srv, Trigger);
  sdescriptor_sync_time.service_name = SERVICE_SYNC_TIME;
  sdescriptor_sync_time.request = &req_sync_time;
  sdescriptor_sync_time.response = &res_sync_time;
  sdescriptor_sync_time.callback = service_sync_time_cb;

  micro_rosso::services.push_back(&sdescriptor_sync_time);

  return true;
}

#endif  // BUILD_SYNC_TIME

