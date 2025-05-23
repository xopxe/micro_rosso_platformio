# micro rosso

This is a modular system for micro-ros under PlatformIO. It allows you to write modules that can interact with ROS2 sending and receiving topics, publishing services, and so on.

For an example of an application that uses micro-rosso, see [oruga](https://github.com/xopxe/micro_rosso_oruga), a tracked robot. In its [platformio.ini](https://github.com/xopxe/micro_rosso_oruga/blob/main/platformio.ini) file, you'll see that it's an ESP32 project that uses Arduino framework, ROS2 jazzy, and depends on [micro-rosso](https://github.com/xopxe/micro_rosso_platformio) library. It provides its functionality in modules like the [mobility system](https://github.com/xopxe/micro_rosso_oruga/tree/main/lib/mobility_tracked). It also depends on external modules, such as [mpu6050](https://github.com/xopxe/micro_rosso_mpu6050) IMU.

## Install

To use micro-rosso, you need a working micro-ros environment and a PlatformIO environment.

First, you will need [ROS2 installed](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).

Then you have to install micro-ros. There are two ways: native build (harder) or a docker image (easier).

### micro-ros from a docker

You can run a micro-ros installation directly from a docker image. For example, when using serial transport to communicate with the microcontroller:

```sh
docker run -it --rm --device=/dev/ttyUSB_ESP32 --net=host microros/micro-ros-agent:jazzy serial --dev /dev/ttyUSB_ESP32 -b 115200
```

You can change the `humble` to `jazzy`or whatever ROS2 version you use.

When using wifi transport:

```sh
docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 2024
```

### micro-ros native build

You can also build the micro-ros environment yourself.

```sh
sudo apt install -y git cmake python3-pip python3-rosdep
mkdir -p ~/microros_ws/src
cd ~/microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build
source install/local_setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source ~/microros_ws/install/local_setup.bash
```

Once micro-ros is installed, when using serial transport to communicate with the microcontroller, you can run it as follows:

```sh
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```

When using wifi transport:

```sh
ros2 run micro_ros_agent micro_ros_agent udp4 --port 2024
```

### PlatformIO environment

You can install PlatformIO inside VSCode following [this](https://docs.platformio.org/en/stable/integration/ide/vscode.html) tutorial.

## Files

The files in the micro_rosso library are:

* `include/micro_rosso.h`, `src/micro_rosso.cpp`: library used to create and run modules.

* `include/micro_rosso_config.h`: internal configuration for `micro_rosso`.

* `include/logger.h`, `src/logger.cpp`: utility module pre-loaded by `micro_rosso` and used to send `/rosout` topics.

* `include/sync_time.h`, `src/sync_time.cpp`: utility module for a service to synchronize the board's clock to the agent.

* `include/ros_status.h`, `src/ros_status.cpp`: utility module that watches the connection status and can provide output, like lighting an LED when the board is connected to the agent.

* `ticker.include/h`, `src/ticker.cpp`: utility module that creates a 1Hz timer and uses it to send to a topic regularly.

## Creating a project

You can create an empty project using the IDE by selecting the framework, board, etc. Then you must edit the `platformio.ini` file to look something like this:

```ini
[env:pico32]
platform = espressif32
board = pico32 ; or whatever esp32 board you have
framework = arduino
board_microros_distro = humble  ; or jazzy, etc.
board_microros_transport = serial

lib_deps = 
    xopxe/micro_rosso
```

This project is developed on ESP32 boards but can be adapted to other Arduino-compatible boards.

A very minimal main.cpp looks like this:

```cpp
#include <Arduino.h>
#include "micro_rosso.h"

void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  if (!micro_rosso::setup( "my_node_name" )) {
    D_println("FAIL micro_rosso.setup()");
  }
}

void loop() {
  micro_rosso::loop();
}
```

The `board_microros_transport` field in the `platformio.ini` file specifies how the board will communicate with the agent. This example uses the Serial transport, which is the first UART or the same USB link used to flash the board. You can [change the transport](https://github.com/micro-ROS/micro_ros_platformio?tab=readme-ov-file#transport-configuration). Each possible transport (serial, Wi-Fi, etc.) must be configured in your code in the `setup()`  method before starting micro_rosso.

## Configuring micro_rosso

The configuration for micro_rosso is stored in [micro_rosso_config.h](include/micro_rosso_config.h) header. Most variables can be manipulated from your `platformio.ini` file through build flags. For example, to use a specific ROS domain you can set:

```ini
build_flags =
    ...
    -DROS_DOMAIN_ID=20
    ...
```

Another important element to configure is the maximum number of publishers, services, subscriptions, etc. For this, first add a `myproject.meta` file, for example:

```yaml
{
    "names": {
        "rmw_microxrcedds": {
            "cmake-args": [
                "-DRMW_UXRCE_MAX_NODES=1",
                "-DRMW_UXRCE_MAX_PUBLISHERS=10",
                "-DRMW_UXRCE_MAX_SUBSCRIPTIONS=10",
                "-DRMW_UXRCE_MAX_SERVICES=5",
                "-DRMW_UXRCE_MAX_CLIENTS=0",
            ]
        }
    }
}
```

Then reference this file from your `platformio.ini` file:

```ini
board_microros_user_meta = myproject.meta
```

## How to use modules

First, we will show how to use a third-party module; later, we will describe how to create your own.

A module is imported as a standard PlatformIO library in any standard way. As an example, we will import the MPU6050 module from GitHub, adding the following [entry](https://docs.platformio.org/en/latest/projectconf/sections/env/options/library/lib_deps.html) to the `platformio.ini` file:

```ini
lib_deps = 
    xopxe/micro_rosso
    https://github.com/xopxe/micro_rosso_mpu6050.git
```

/TIP: include from a local folder to reduce rebuilding time/

To start a module, you must include and, if needed, configure it in your `main.cpp`. Following the mpu6050 example, add the following somewhere near the top and after the `#include "micro_rosso.h"`:

```cpp
#include "micro_rosso_mpu6050.h"
ImuMPU6050 imu;
```

Then, call the initialization in the setup function:

```cpp
void setup() {
  ...
  if (!imu.setup()) {
    D_println("FAIL imu.setup()");
  };
}
```

Check the `setup()` call to see what optional parameter you can pass to it, such as the I2C channel, topic names, etc.

Usually, the modules' setup method returns `false` if something fails. `D_print` and `D_println` are macros that print to a debug console. You can configure the serial debug consoles by passing build flags from your project's `platformio.ini` as described above. For example:

```ini
build_flags =
    -DDEBUG_CONSOLE=Serial1
    -DDEBUG_CONSOLE_PIN_RX=10
    -DDEBUG_CONSOLE_PIN_TX=9
    -DDEBUG_CONSOLE_BAUD=115200
    ...
```

/TIP: if RX or TX are not defined, then the default pins for the used Serial will be used.

## How to write a module

A micro\_rosso module is a mostly static object that provides a setup method where it registers ros2 resources using `micro_rosso.h`. It then uses micro_ros_platformio and other modules to implement its functionality. You have various options for where to place libraries in your project:

* Both .h and .cpp files in the src/ directory. (quick and dirty)
* The .h in the include/ folder, the .cpp in src/ (good for when you are writing a library you will publish)
* In a lib/my_module/ folder (good for private modules that only make sense for your project.)

Things a module can do:

### Publish topics

The following example is derived from the `ticker` module. It will publish (by default)`/tick` topics, of type `int32`.

In `my_module.cpp` create and register the publisher. We will create a static object to store the message to be sent and a publisher object:

```cpp
static std_msgs__msg__Int32 msg_tick;
static publisher_descriptor my_topic;
```

Then, in the setup method, we initialize and register the publisher object :

```cpp
  msg_tick.data = 0; // also initialize the topic message as needed
  my_topic.qos = QOS_DEFAULT; // can also be QOS_BEST_EFFORT or QOS_CUSTOM
  my_topic.type_support = 
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);
  my_topic.topic_name = "/tick";
  micro_rosso::publishers.push_back(&my_topic);
```

The topic then can be published from methods, timers, event handlers, etc., as follows:

```cpp
  rcl_publish(&my_topic.publisher, &msg_tick, NULL);
  msg_tick.data++;
```

You can also use custom QoS setting `my_topic.qos = QOS_CUSTOM;` and provide a QoS profile in the `my_topic.qos_profile` field.

### Subscribe to topics

The following example is derived from the [`mobility_tracked`](https://github.com/xopxe/micro_rosso_oruga/tree/main/lib/mobility_tracked) module. It will subscribe to `/cmd_vel` topics of type `cmd_vel`.

In the `my_module.h` file, create the module class:

```h
class MyModule {
  static bool setup();
}
```

In `my_module.cpp`, create and register the subscription. We will create a static object to store the messages when they arrive and a subscription object:

```cpp
static geometry_msgs__msg__Twist msg_twist;
static subscriber_descriptor my_subscription; 
```

Then, define a method to attend to the messages when they arrive:

```cpp
static void cb(const void* msgin) {
  // We can read the message from the global message object 
  D_println(msg_twist.linear.x);
  
  // or we can retrieve the object from the call, which is
  // useful when sharing the callback between topics:
  // const geometry_msgs__msg__Twist* msg = 
  //   (const geometry_msgs__msg__Twist*)msgin;
}
```

Finally, in the setup method, we initialize and register the subscription object:

```cpp
bool MyModule::setup() {
  my_subscription.qos = QOS_DEFAULT;
  my_subscription.type_support = 
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist);
  my_subscription.topic_name = "/cmd_vel";
  my_subscription.msg = &msg_twist;
  my_subscription.callback = &cb;
  micro_rosso::subscribers.push_back(&my_subscription_cmd_vel);
  return true;
}
```

The QoS functions similarly to how it does in publishers.

### Use and register timers

`micro_rosso` provides two timers by default, `micro_rosso::timer_control` at 50Hz (20ms period) and `micro_rosso::timer_report` at 5Hz (200ms period). To use a timer, create a callback function and add it to the timer's callbacks vector:

```cpp
void report_cb(int64_t last_call_time) {
  D_println("called at 5Hz!");
}

bool setup() {
  ...
  micro_rosso::timer_report.callbacks.push_back(&report_cb);
  ...
}
```

The `last_call_time` parameter is the time since the last time the timer triggered, in nanoseconds.

It is possible to create new timers. For an example, see the `ticker` module, which creates and provides a 1Hz timer. To do this, first, instantiate a timer descriptor. If you want to make it usable by other modules, you can do it in the class definition.

```cpp
class MyModule {
  static bool setup();
  static timer_descriptor my_timer;
}
```

Create a timer handler function in the `.cpp` file. The timer descriptor has a callback vector property, so a basic handler will have to call all the registered callbacks:

```cpp
static void timer_handler (rcl_timer_t* timer, int64_t last_call_time) {
  for (int i = 0; i < Ticker::my_timer.callbacks.size(); i++) {
    Ticker::my_timer.callbacks[i](last_call_time);
  }
  return;
}
```

Finally, in the setup method, initialize the timer descriptor object and register it.

```cpp
bool MyModule::setup() {
  ...
  Ticker::timer_tick.timeout_ns = RCL_MS_TO_NS(1000); // 1 sec
  Ticker::timer_tick.timer_handler = timer_handler;
  micro_rosso::timers.push_back(&Ticker::timer_tick);
  ...
}
```

### Serve services

See the example in the `sync_time` module for creating and announcing a service. First, create a service descriptor and the request and response objects:

```cpp
static service_descriptor my_service;
std_srvs__srv__Trigger_Request req_service;
std_srvs__srv__Trigger_Response res_service;
```

Then, create the callback function for the service:

```cpp
static void service_cb(const void* req, void* res) {
  // request and response can be cast from the call:
  // std_srvs__srv__Trigger_Response* res_in = 
  //   (std_srvs__srv__Trigger_Response*)res;

  res_service.success = micro_rosso::time_sync(); // implement the service, return status
}
```

Finally, configure and register the service descriptor:

```cpp
  my_service.qos = QOS_DEFAULT; // can also be QOS_BEST_EFFORT
  my_service.type_support =
    ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger);
  my_service_sync_time.service_name = "/my_service_name";
  my_service.request = &req_service;
  my_service.response = &res_service;
  my_service.callback = service_cb;

  micro_rosso::services.push_back(&my_service);
```

### Subscribe to ROS state events

You can respond to ROS state changes, for example, by disabling and enabling hardware when micro-ros get connected or disconnected. For that, you must register an appropriate listener:

```cpp
static void ros_state_cb(ros_states state) {
  switch (state) {
    case WAITING_AGENT:      // the client is waiting the agent
      break;
    case AGENT_AVAILABLE:    // agent detected, connecting
      break;
    case AGENT_CONNECTED:    // client connected to agent
      break;
    case AGENT_DISCONNECTED: // connection to agent broken, disconnecting
      break;
    default:
      break;
  }
}

bool MyModule::setup() {
  ...
  micro_rosso::ros_state_listeners.push_back(ros_state_cb);
  ...
  return true;
}
```

### Post-init callback

If your module needs to do something after ROS has been initialized, like publish some topics, you can add a callback:

```cpp
static void on_start() {
  micro_rosso::logger.log("MyModule running"); // sends /rosout topic
}

bool MyModule::setup() {
  ...
  micro_rosso::post_init.push_back(on_start);
  ...
  return true;
}
```

### Use the parameter server

You can enable the micro-ros [parameter server](https://micro.ros.org/docs/tutorials/programming_rcl_rclc/parameters/) by setting the appropriate compiler flag in your `platformio.ini` file:

```ini
build_flags =
    -DROS_PARAMETER_SERVER=true
```

Notice that the parameter server uses additional slots (6?) from the available services. To increase the number of services, see the "Configuring micro_rosso" section above.

To react to the creation, modification, and removal of configuration parameters, you must subscribe to the `parameter_change_listeners` list:

```cpp
static void parameter_change_cb(const Parameter *old_param, const Parameter *new_param) {
  if (old_param == NULL) {
    if (new_param != NULL && strcmp(new_param->name.data, "parameter1") == 0) {
      // parameter1 created
    }
  } else {
    if (new_param == NULL) {
      // parameter1 deleted
    } else if (strcmp(new_param->name.data, "parameter1") == 0) {
      // parameter1 updated
    }
  }
}

bool MyModule::setup()
{
  ...
  micro_rosso::parameter_change_listeners.push_back(parameter_change_cb);
  ...
  return true;
}
```

You might want to create the variables from inside your firmware and give them initial values. You must do that from a ros status change event once the ros is connected. For that, you must register a ros status listener as described above and add the parameter creation code:

```cpp
static void ros_state_cb(ros_states state)
{
  switch (state)
  {
  case AGENT_CONNECTED:
    rclc_add_parameter(&micro_rosso::param_server, "parameter1", RCLC_PARAMETER_INT);
    rclc_parameter_set_int(&micro_rosso::param_server, "parameter1", 10);
    break;
  case AGENT_DISCONNECTED:
    rclc_delete_parameter(&micro_rosso::param_server, "parameter1");
    break;
  default:
    break;
  }
}
```

Remember that the micro-ros parameter server only supports `int64`, `double`, and `bool` parameters.

### Enabling persistence for the parameter server

The `parameter_persist` module allows parameter values to be preserved across board reboots. To enable it, load the module the usual way. In your `main.cpp` add:

```cpp
#include "parameter_persist.h"
ParameterPersist persist;
...
void setup() {
  ...
  if (!persist.setup()) {
    D_println("FAIL persist.setup()");
  };
}
```

You can specify what parameters to persist using two attributes in the parameter_persits modules:

* `bool ParameterPersist::persist_all`: if set to true, all parameters will be persisted. If set to false, only the ones in the following list will:

* `std::vector<char *> ParameterPersist::persist_list`: a list of strings with the names of the parameters to persist.

When using persistence, you must assign a default value to a parameter only when it is not already initialized from flash:

```cpp
static void ros_state_cb(ros_states state)
{
  int rc;
  switch (state)
  {
  case AGENT_CONNECTED:
    rc = rclc_add_parameter(&micro_rosso::param_server, "parameter1", RCLC_PARAMETER_INT);
    if (rc == 0) {
      // the parameter was added successfully, so it wasn't created before. Assign a value:
      RCNOCHECK(rclc_parameter_set_int(&micro_rosso::param_server, "parameter1", 10));
    }
    break;
  case AGENT_DISCONNECTED:
    rclc_delete_parameter(&micro_rosso::param_server, "parameter1");
    break;
  default:
    break;
  }
}
```

NOTE: This service uses the portable Arduino `Preferences` library. Unfortunately, this library does not support discovering the data stored on Flash. So, for this purpose the service uses the ESP32native nvs.h library. If you are porting this system to another platform, you will have to replace that code.

### TODO consume services

## Example commands

```sh
picocom --baud 115200 /dev/ttyUSB1
ros2 topic list
ros2 topic echo "/imu/raw"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
ros2 service call /sync_time std_srvs/srv/Trigger "{}"
ros2 topic echo /rosout --field msg
```

## Authors and acknowledgment

<jvisca@fing.edu.uy> - [Grupo MINA](https://www.fing.edu.uy/inco/grupos/mina/), Facultad de Ingeniería - Udelar, 2024

## License

Apache 2.0
