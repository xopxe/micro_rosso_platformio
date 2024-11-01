#ifndef __micro_rosso_config_h
#define __micro_rosso_config_h

// #define ROS2_NODE_NAME "micro_rosso_rclc"

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

#endif // __micro_rosso_config_h
