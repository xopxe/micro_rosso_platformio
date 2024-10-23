#ifndef __Mobility_tracked_config_h
#define __Mobility_tracked_config_h

#define STOP_TIMEOUT_MS 2000

// robot specs from https://wiki.lynxmotion.com/info/wiki/lynxmotion/view/rover-kits/a4wd3-tracked/
static const float MOTOR_MAX_RPM = 170.0;   // motor's max RPM
static const float MAX_RPM_RATIO = 0.85;    // max RPM allowed (under load reported as 145rpm)
static const float MOTOR_REDUCTION = 51;
static const float MAX_RPM_ALLOWED = MOTOR_MAX_RPM * MAX_RPM_RATIO;
static const unsigned int ENCODER_PPM = 12;                        // encoder resolution
static const float WHEEL_DIAMETER = 0.202;                         // wheel's diameter in meters
static const float PULSES_PER_REV = ENCODER_PPM * MOTOR_REDUCTION; // encoder ticks per wheel rev - 612
static const float WHEEL_RADIUS = WHEEL_DIAMETER / 2;
static const float LR_WHEELS_DISTANCE = (0.4515 + 0.2915) / 2;// distance between left and right wheels

// compute dynamics from motors and geometry
static const float MAX_WHEEL_ANGULAR = MAX_RPM_ALLOWED * 2 * PI / 60;// rad/s
static const float MAX_SPEED = MAX_WHEEL_ANGULAR * WHEEL_RADIUS;     // ~1.5   (datasheet says ~1.8m/s)
static const float MAX_TURNSPEED = atan(2 * MAX_SPEED / LR_WHEELS_DISTANCE);  // rad/s

// Mas speed to use under joystick operation
static const float MAX_JOY_SPEED = MAX_SPEED;
static const float MAX_JOY_TURNSPEED = MAX_TURNSPEED;
//#define JOY_REGULATED         // if defined, speed is pid-regulated, otherwise go directly to motors.

// Pid for linear velocity, from speed in m/s to power level (-126..126)
static const float PID_LINEAL_KF = 126 / MAX_SPEED;      // feed forward control
static const float PID_LINEAL_KP = 0.0 * PID_LINEAL_KF;  // P control
static const float PID_LINEAL_KI = -0.0;                  // I control
static const float PID_LINEAL_KD = 0.0;                  // D control

// Pid for rotation velocity, from rad/s to turn power level (-126..126)
static const float PID_TURN_KF = 126 / MAX_TURNSPEED;  // feed forward control
static const float PID_TURN_KP = 0.0 * PID_TURN_KF;    // P control
static const float PID_TURN_KI = -0.0;                  // I control
static const float PID_TURN_KD = 0.0;                  // D control

// Wiring pins
#define SABERTOOTH_SERIAL Serial2
#define SABERTOOTH_TX_PIN 27

// left side encoders A&B pin definition swapped to reverse direction
#define ENCODER_rr_lft_PIN_A 26
#define ENCODER_rr_lft_PIN_B 25
#define ENCODER_fr_lft_PIN_A 18
#define ENCODER_fr_lft_PIN_B 5
#define ENCODER_fr_rgt_PIN_A 23
#define ENCODER_fr_rgt_PIN_B 19
#define ENCODER_rr_rgt_PIN_A 36
#define ENCODER_rr_rgt_PIN_B 39

#endif  // __Mobility_tracked_config_h
