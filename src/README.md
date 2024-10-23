# micro rosso

This is a modular system for Arduino micro-ros. It allows you to write modules that can interact with ROS2 sending and receiving topics, publishing services, and so on.

## Install

**micro-ros**

Install [Arduino micro-ros](https://www.hackster.io/514301/micro-ros-on-esp32-using-arduino-ide-1360ca) environment.

Another handy tutorial [here](https://medium.com/@kabilankb2003/seamless-communication-between-jetson-nano-and-esp32-with-microros-bd82c1cc7c53)

**Board support for Arduino**

This project is developed on ESP32 boards but can easily be adapted to other Arduino-compatible boards. 

* esp32 by Esspressif.

Note: when using ESP32 boards you might have to stay at v2.0.17

Note: In Arduino micro-ros, the maximum number of resources (services, publishers, subscribers, timers, etc.) is configured in the `colcon.meta` file.


**Arduino libraries**

These are libraries needed by the base system.

* [micro_ros_arduino](https://github.com/micro-ROS/micro_ros_arduino): ROS2 API. Must be installed into the sketch from the zip file downloaded from the [releases](https://github.com/micro-ROS/micro_ros_arduino/releases) page. Select the one corresponding to you ROS2 version (e.g. humble, iron)


## Files

* `micro_rosso.ino`: Main sketch, modify to include and setup modules.

* `micro_rosso.h`, `micro_rosso.cpp`: library used to create and run modules.

* `micro_rosso_config.h`: configuration for `micro_rosso`. There you can configuire the ros2 node name, i2c pins, debugging console output, micro-ros transport, etc.

* `logger.h`, `logger.cpp`: utility module pre-loaded by `micro_rosso` and used to send `/rosout` topics.

* everything else in the `src/` folder: modules and their dependencies.



## Configuring micro_rosso 

You can configure micro_rosso editing `micro_rosso_config.h` before building your project.


## Provided Modules

Different modules have their own dependencies. If you do not want to build a module, comment the corresponding `#define` in `src/build_modules.h`.

* `env_bme680` environmental sensor. Depends on the Adafruit BME680 library by Adafruit.

* `env_dht22` environmental sensor.

* `imu_bno08x` IMU. Depends on the SparkFun BNO08X Cortex Based IMU library by SparkFun Electronics

* `imu_mpu6050`IMU. Depends on the Adafruit MPU6050 library by Adafruit.

* `odom_helper` odometry library. Simple dead-reconing tracker and ROS2 odometry topic publisher.

* `ros_status` module, allows to react to changes in the connection status (e.g., power on a LED when an agent is connected).

* `sync_time` service. Sinchronize the microcontroller's clock to ROS2. Depends on the Time library by Michael Margolis. It's not strictly necessary and can be disabled by commenting the `#define USE_SET_TIME` in `micro_rosso_config.h`.

* `ticker` publisher. Cerates a 1Hz timer.

* `robotito/robotito_omni` module for the/robotito/ platform. Omnidirectional platform using two dual H-bridges. Depends on the Cdrv8833 library by Steffano Ledda, and the ESP32Encoder library by Kevin Harrington.

* `robotito/robotito_apds9960` module for the/robotito/ platform. Supports the APDS9960 color/proximity/gesture sensor. Depends on the Arduino\_APDS9960 library by Arduino.

* `robotito/robotito_vl53ring` module for the/robotito/ platform. Produces a laser\_scan fro the ring of VL53L0X TOF distance sensors. Depends on the Adafruit VL53L0X library by Adafruit.

* `oruga/mobility_tracked` module for a tracked robot usng a Sabertooth motor controller. Depends on the ESP32Encoder library by Kevin Harrington. 



## How to use modules

To start a module, you must include and, if needed, configure it in `micro_rosso.ino`. As a typical example for the module for the mpu6050 imu, add the following somewhere near the top and after the `#include "micro_rosso.h"`: 

```
#include "imu_mpu6050.h"
ImuMPU6050 imu;
```

Then, call the initialization in the setup function:

```
void setup() {
  ...
  if (!imu.setup()) {
    D_println("FAIL imu.setup()");
  };
}
```

Usually, modules' setup method return `false` if something fails. `D_print` and `D_println` are macros that print to a debug console (see `DEBUG_CONSOLE` macros in `micro_rosso.h` to see if it is enabled and where it goes to).


## How to write a module

A micro\_rosso module is a mostly static object that provides a setup method where it registers ros2 resources using `micro_rosso.h`. It then uses ros2arduino and other modules to implement its own functionality.

Things a module can do:

### Subscribe to topics

The following example is derived from the `mobility_tracked` module. It will subscribe to `/cmd_vel` topics of type `cmd_vel`.

In the `my_module.h` file, create the module class:

```
class MyModule {
  static bool setup();
}
```

In `my_module.cpp`, create and register the subscription. We will create a static object to store the messages when they arrive, and a subscription object:

```
static geometry_msgs__msg__Twist msg_twist;
static subscriber_descriptor my_subscription; 
```

Then, define a method to attend to the messages when they arrive:

```
static void cb(const void* msgin) {
  // we can read the message from the global message object 
  D_println(msg_twist.linear.x);
  
  // or we can retrieve the object from the call, useful when 
  // sharing the callback between topics:
  // const geometry_msgs__msg__Twist* msg = 
  //   (const geometry_msgs__msg__Twist*)msgin;
}
```

Finally, in the setup method we initialize and register the subscription object:

```
bool MyModule::setup() {
  my_subscription.type_support = 
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist);
  my_subscription.topic_name = "/cmd_vel";
  my_subscription.msg = &msg_twist;
  my_subscription.callback = &cb;
  micro_rosso::subscribers.push_back(&my_subscription_cmd_vel);
  return true;
}

```

### Publish topics

The following example is derived from the `ticker` module. It will publish to `/tick` topics of type `int32`.

In `my_module.cpp` create and register the publisher. We will create a static object to store the message to be sent and a publisher object:

```
static std_msgs__msg__Int32 msg_tick;
static publisher_descriptor my_topic;
```

Then in the setup method we initialize and register the publisher object :

```
  msg_tick.data = 0; // also initialize the topic message as needed
  my_topic.qos = QOS_DEFAULT; // can also be QOS_BEST_EFFORT
  my_topic.type_support = 
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);
  my_topic.topic_name = "/tick";
  micro_rosso::publishers.push_back(&my_topic);
```

The topic then can be published from methods, timers, event handlers, etc., as follows:

```
  rcl_publish(&my_topic.publisher, &msg_tick, NULL);
  msg_tick.data++;
```

### Use and register timers

`micro_rosso` provides two timers by default, `micro_rosso::timer_control` at 50Hz (20ms period) and `micro_rosso::timer_report` at 5Hz (200ms period). To use a timer create a callback function and add it to the timer's callbacks vector:

```
void report_cb(int64_t last_call_time) {
  D_println("called at 5Hz!");
}

bool setup() {
  ...
  micro_rosso::timer_report.callbacks.push_back(&report_cb);
  ...
}
```

It is possible to create new timers. For an example, see the `ticker` module, which creates and provides a 1Hz timer.For this, first instantiate a timer descriptor. You can do it in the class definition if you want to make it useable by other modules.

```
class MyModule {
  static bool setup();
  static timer_descriptor my_timer;
}
```

Create a timer handler function in the `.cpp` file. The timer descriptor has a callback vector property, so a basic handler will have to call all the registered callbacks:

```
static void timer_handler (rcl_timer_t* timer, int64_t last_call_time) {
  for (int i = 0; i < Ticker::my_timer.callbacks.size(); i++) {
    Ticker::my_timer.callbacks[i](last_call_time);
  }
  return;
}
```

Finally, in the setup method initialize the timer descriptor object and register it.

```
bool MyModule::setup() {
  ...
  Ticker::timer_tick.timeout_ns = RCL_MS_TO_NS(1000); // 1 sec
  Ticker::timer_tick.timer_handler = timer_handler;
  micro_rosso::timers.push_back(&Ticker::timer_tick);
  ...
}
```

### Serve services

To create and announce a service, see the example in the `sync_time` module. First, create a service descriptor and the request and response objects:

```
static service_descriptor my_service;
example_interfaces__srv__Trigger_Request req_service;
example_interfaces__srv__Trigger_Response res_service;
```

Then, create the callback function for the service:

```
static void service_cb(const void* req, void* res) {
  // request and response can be cast from the call:
  // example_interfaces__srv__Trigger_Response* res_in = 
  //   (example_interfaces__srv__Trigger_Response*)res;

  res_service.success = micro_rosso::time_sync(); // implement the service, return status
}
```

Finally, configure and register the service descriptor:

```
  my_service.qos = QOS_DEFAULT; // can also be QOS_BEST_EFFORT
  my_service.type_support =
    ROSIDL_GET_SRV_TYPE_SUPPORT(example_interfaces, srv, Trigger);
  my_service_sync_time.service_name = "/my_service_name";
  my_service.request = &req_service;
  my_service.response = &res_service;
  my_service.callback = service_cb;

  micro_rosso::services.push_back(&my_service);
```

### Subscribe to ROS state events

You can respond to ROS state changes, for example by disabling and enabling hardware when micro-ros gets connected or diconnected. For that you must register an appropiate listener:

```
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
 


### TODO consume services


## Example commands

```
$ ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
$ ros2 run micro_ros_agent micro_ros_agent udp4 --port 2024
$ picocom --baud 115200 /dev/ttyUSB1
$ ros2 topic list
$ ros2 topic echo "/imu/raw"
$ ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
$ ros2 service call /sync_time example_interfaces/srv/Trigger "{}"
$ ros2 topic echo /rosout --field msg
```



## Authors and acknowledgment

jvisca@fing.edu.uy - [Grupo MINA](https://www.fing.edu.uy/inco/grupos/mina/), Facultad de Ingeniería - Udelar, 2024

## License

MIT


