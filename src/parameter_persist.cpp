#if ROS_PARAMETER_SERVER

#include <Arduino.h>
#include <Preferences.h>
#include <nvs.h>

#include "micro_rosso.h"

#include "parameter_persist.h"

static const char *name_space = "micro_rosso";

static char *loading_from_flash = NULL;

std::vector<char *> ParameterPersist::persist_list;
bool ParameterPersist::persist_all = true;
bool ParameterPersist::readonly = false;

Preferences preferences;

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

ParameterPersist::ParameterPersist()
{
}

static void load_variables()
{
  nvs_iterator_t it = nvs_entry_find(NVS_DEFAULT_PART_NAME, name_space, NVS_TYPE_ANY);
  while (it != NULL)
  {
    nvs_entry_info_t info;
    nvs_entry_info(it, &info);
    it = nvs_entry_next(it);
    D_print("NVS found: key '");
    D_print(info.key);
    D_println("'");

    loading_from_flash = info.key;

    preferences.begin(name_space, ParameterPersist::readonly);

    switch (info.type)
    {
      int rc;
      bool value_bool;
      int64_t value_int64;
      double value_double;
    case NVS_TYPE_U8: // bool
      value_bool = preferences.getBool(info.key);
      rc = rclc_add_parameter(&micro_rosso::param_server, info.key, RCLC_PARAMETER_DOUBLE);
      rc |= rclc_parameter_set_bool(&micro_rosso::param_server, info.key, value_bool);
      if (rc != 0)
      {
        D_println("NVS failed reading boolean from flash");
      }
      break;
    case NVS_TYPE_I64:
      value_int64 = preferences.getLong64(info.key);
      rc = rclc_add_parameter(&micro_rosso::param_server, info.key, RCLC_PARAMETER_INT);
      rc |= rclc_parameter_set_int(&micro_rosso::param_server, info.key, value_int64);
      if (rc != 0)
      {
        D_println("NVS failed reading int64 from flash");
      }
      break;
    case NVS_TYPE_BLOB: // double
      value_double = preferences.getDouble(info.key);
      rc = rclc_add_parameter(&micro_rosso::param_server, info.key, RCLC_PARAMETER_DOUBLE);
      rc |= rclc_parameter_set_double(&micro_rosso::param_server, info.key, value_double);
      if (rc != 0)
      {
        D_println("NVS failed reading double from flash");
      }
      break;
    default:
      break;
    }
  };
  // Note: no need to release iterator obtained from nvs_entry_find function when
  //       nvs_entry_find or nvs_entry_next function return NULL, indicating no other
  //       element for specified criteria was found.

  preferences.end();
  loading_from_flash = NULL;
}

static void persist_variable(const Parameter *p)
{
  // we got here looping from load_variables() -> micro_rosso::on_parameter_changed() -> persist_variable()
  // no need to persist, we just read the value from flash.
  if (loading_from_flash != NULL && strcmp(loading_from_flash, p->name.data) == 0)
  {
    return;
  }

  D_print("persisting: ");
  D_print(p->name.data);

  preferences.begin(name_space, ParameterPersist::readonly);

  switch (p->value.type)
  {
  case RCLC_PARAMETER_BOOL:
    D_print("(bool) ");
    D_println(p->value.bool_value);
    preferences.putBool(p->name.data, p->value.bool_value);
    break;
  case RCLC_PARAMETER_INT:
    D_print("(int64) ");
    D_println(p->value.integer_value);
    preferences.putLong64(p->name.data, p->value.integer_value);
    break;
  case RCLC_PARAMETER_DOUBLE:
    D_print("(double) ");
    D_println(p->value.double_value);
    preferences.putDouble(p->name.data, p->value.double_value);
    break;
  default:
    D_println(" unsupported type: ");
    D_println(p->value.type);
    break;
  }

  preferences.end();
}

static void unpersist_variable(const char *name)
{
  D_print("unpersisting: ");
  D_println(name);
  preferences.begin(name_space, ParameterPersist::readonly);
  preferences.remove(name);
  preferences.end();
}

static void ros_state_cb(ros_states state)
{
  int rc;
  switch (state)
  {
  case AGENT_CONNECTED: // client connected to agent
    load_variables();
    break;
  case AGENT_DISCONNECTED: // connection to agent broken, disconnecting
    break;
  default:
    break;
  }
}

static bool in_persist_list(const char *key)
{
  for (int i = 0; i < ParameterPersist::persist_list.size(); i++)
  {
    if (strcmp(key, ParameterPersist::persist_list[i]) == 0)
    {
      return true;
    }
  }
  return false;
}

static void parameter_change_cb(const Parameter *old_param, const Parameter *new_param)
{
  if (old_param == NULL)
  {
    // if (new_param != NULL && strcmp(new_param->name.data, "parameter1") == 0)
    if (new_param != NULL && (ParameterPersist::persist_all || in_persist_list(new_param->name.data)))
    {
      persist_variable(new_param);
    }
  }
  else
  {
    if (new_param == NULL)
    {
      unpersist_variable(old_param->name.data);
    }
    else
    {
      if (ParameterPersist::persist_all || in_persist_list(new_param->name.data))
      {
        persist_variable(new_param);
      }
    }
  }
}

bool ParameterPersist::setup(bool readonly)
{
  D_println("setup: parameter_persist");

  micro_rosso::ros_state_listeners.push_back(ros_state_cb);
  micro_rosso::parameter_change_listeners.push_back(parameter_change_cb);

  return true;
}

#endif //ROS_PARAMETER_SERVER