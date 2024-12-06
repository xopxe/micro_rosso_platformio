#ifndef __micro_rosso_config_h
#define __micro_rosso_config_h

/*
If you want to change these values, pass the adecuate defines when building.
For example, to enable serial debugging add the  following in the platfomrio.ini
file for the project that loads this library:

build_flags =
    -DDEBUG_CONSOLE=Serial1
    -DDEBUG_CONSOLE_PIN_RX=10
    -DDEBUG_CONSOLE_PIN_TX=9
    -DDEBUG_CONSOLE_BAUD=115200

*/

/*
 Serial console used for debug output. If commented, no output is prdouced.
 To send to USB serial, set DEBUG_CONSOLE ro Serial and PINs to -1.
*/
// #define DEBUG_CONSOLE Serial1
// #define DEBUG_CONSOLE_PIN_RX 10
// #define DEBUG_CONSOLE_PIN_TX 9
// #define DEBUG_CONSOLE_BAUD 115200

/*
 Set the time on the microconroller from ROS, useful
 for non-ROS code that might whant to know the time.
*/
#if !defined(USE_SET_TIME)
#define USE_SET_TIME true
#endif

/*
 The two default timers created, one for control tasks
 (e.g. pid), and another for reporting (e.g. odom)
*/
#if !defined(TIMER_CONTROL_MS)
#define TIMER_CONTROL_MS 50 // 20Hz
#endif
#if !defined(TIMER_REPORT_MS)
#define TIMER_REPORT_MS 200 // 5Hz
#endif

/*
 Set to true to enable the parameter server
*/
#if !defined(ROS_PARAMETER_SERVER)
#define ROS_PARAMETER_SERVER false
#endif
/*
 The configuration for the parameter server (used if enabled)
*/
#if ROS_PARAMETER_SERVER
#include <rclc_parameter/rclc_parameter.h>
static const rclc_parameter_options_t parameter_options = {
    .notify_changed_over_dds = true,
    .max_params = 4,
    .allow_undeclared_parameters = true,
    .low_mem_mode = false};
#endif

#endif // __micro_rosso_config_h
