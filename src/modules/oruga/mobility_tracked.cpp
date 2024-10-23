#include "../build_modules.h"
#ifdef BUILD_MOBILITY_TRACKED

#include "micro_rosso.h"

#include "mobility_tracked_config.h"
#include "mobility_tracked.h"

#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/joy.h>

static geometry_msgs__msg__Twist msg_cmd_vel;
static sensor_msgs__msg__Joy msg_joy;
static subscriber_descriptor sdescriptor_cmd_vel;
static subscriber_descriptor sdescriptor_joy;

#include "../sabertooth.h"
#include <ESP32Encoder.h>
#include "../pid.h"
#include "../odom_helper.h"

static const float TICS_TO_RAD = 2 * PI / PULSES_PER_REV;

static Sabertooth sabertooth(128, SABERTOOTH_SERIAL);

static unsigned long set_control_time_ms = 0;

static ESP32Encoder encoder_rr_lft;
static ESP32Encoder encoder_fr_lft;
static ESP32Encoder encoder_fr_rgt;
static ESP32Encoder encoder_rr_rgt;
static unsigned long last_encoder_us = 0UL;

static Pid pid_advance(-126, 126, PID_LINEAL_KF, PID_LINEAL_KP, PID_LINEAL_KI, PID_LINEAL_KD);
static Pid pid_turn(-126, 126, PID_TURN_KF, PID_TURN_KP, PID_TURN_KI, PID_TURN_KD);

static OdomHelper odom;

float current_linear;
float current_angular;

static float target_linear;
static float target_angular;

static float current_linear_left;
static float current_linear_right;

static float last_dt_s;

static float wheel_angular_rr_lft;
static float wheel_angular_fr_lft;
static float wheel_angular_fr_rgt;
static float wheel_angular_rr_rgt;

static int64_t enc_count_rr_lft = 0;
static int64_t enc_count_fr_lft = 0;
static int64_t enc_count_fr_rgt = 0;
static int64_t enc_count_rr_rgt = 0;

class DisableInterruptsGuard {
public:
  DisableInterruptsGuard() {
    noInterrupts();
  }

  ~DisableInterruptsGuard() {
    interrupts();
  }
};

MobilityTracked::MobilityTracked(){
  //(void*)TAG_ENCODER;
};

static void tracked_set_target_velocities(float linear, float angular) {
  if (linear > MAX_SPEED) linear = MAX_SPEED;
  else if (linear < -MAX_SPEED) linear = -MAX_SPEED;
  if (angular > MAX_TURNSPEED) angular = MAX_TURNSPEED;
  else if (angular < -MAX_TURNSPEED) angular = -MAX_TURNSPEED;

  if (target_linear != linear) {
    //TODO pid reset?
    target_linear = linear;
  }
  if (target_angular != angular) {
    //TODO pid reset?
    target_angular = angular;
  }
}

static void set_target_velocities(const geometry_msgs__msg__Twist& msg_cmd_vel) {
  set_control_time_ms = millis();

  float linear = msg_cmd_vel.linear.x;
  float angular = msg_cmd_vel.angular.z;

  tracked_set_target_velocities(linear, angular);
}

static void set_target_joy(const sensor_msgs__msg__Joy& msg_joy) {
  set_control_time_ms = millis();

  float advance = msg_joy.axes.data[0];
  float turn = msg_joy.axes.data[1];
  /*
  D_print("TOPIC joy: axes (");
  D_print(msg_joy.axes.size);
  D_print(") [0]");
  D_print(msg_joy.axes.data[0]);
  D_print(" [1]");
  D_print(msg_joy.axes.data[1]);
  D_print(" buttons (");
  D_print(msg_joy.buttons.size);
  D_print(") [0]");
  D_println(msg_joy.buttons.data[0]);
  */

#ifdef JOY_REGULATED
  tracked_set_target_velocities(advance * MAX_JOY_SPEED, turn * MAX_JOY_TURNSPEED);
#else
  //TODO disable motor pid regulataion (with a timeout?)
  sabertooth.drive((int8_t)(advance * MAX_JOY_SPEED));
  sabertooth.turn((int8_t)(turn * MAX_JOY_TURNSPEED));
#endif
}

static void compute_movement() {
  unsigned long now;
  int64_t count_fr_lft;
  int64_t count_fr_rgt;
  int64_t count_rr_lft;
  int64_t count_rr_rgt;
  {
    volatile DisableInterruptsGuard interrupt;
    now = micros();
    count_fr_lft = encoder_fr_lft.getCount();
    count_fr_rgt = encoder_fr_rgt.getCount();
    count_rr_lft = encoder_rr_lft.getCount();
    count_rr_rgt = encoder_rr_rgt.getCount();
  }

  uint16_t dt_us = now - last_encoder_us;
  last_dt_s = ((float)dt_us) / 1000000.0;
  last_encoder_us = now;

  // rad/s
  float tics__to__rad_s = 1000000.0 * TICS_TO_RAD / dt_us;
  wheel_angular_fr_lft = tics__to__rad_s * (count_fr_lft - enc_count_fr_lft);
  wheel_angular_fr_rgt = tics__to__rad_s * (count_fr_rgt - enc_count_fr_rgt);
  wheel_angular_rr_lft = tics__to__rad_s * (count_rr_lft - enc_count_rr_lft);
  wheel_angular_rr_rgt = tics__to__rad_s * (count_rr_rgt - enc_count_rr_rgt);

/*
D_println(dt_us);
D_print((1000000.0*(count_fr_lft - enc_count_fr_lft))/dt_us);
D_print(" ^ ");
D_println((1000000.0*(count_fr_rgt - enc_count_fr_rgt))/dt_us);
D_print(((float)(count_rr_lft - enc_count_rr_lft))/dt_us);
D_print(" v ");
D_println(((float)(count_rr_rgt - enc_count_rr_rgt))/dt_us);
*/

/*
D_println(dt_us);
D_print(wheel_angular_fr_lft);
D_print(" ^ ");
D_println(wheel_angular_fr_rgt);
D_print(wheel_angular_rr_lft);
D_print(" v ");
D_println(wheel_angular_rr_rgt);
*/

  enc_count_fr_lft = count_fr_lft;
  enc_count_fr_rgt = count_fr_rgt;
  enc_count_rr_lft = count_rr_lft;
  enc_count_rr_rgt = count_rr_rgt;

  current_linear_left = WHEEL_RADIUS * (wheel_angular_rr_lft+wheel_angular_fr_lft) / 2;  //FIXME
  current_linear_right = WHEEL_RADIUS * (wheel_angular_rr_rgt+wheel_angular_fr_rgt) / 2;

/*
D_print(current_linear_left);
D_print(" L ");
D_println(current_linear_right);
*/

}

static void control_cb( int64_t last_call_time ) {
  //D_print(".");
  
  odom.update_pos(current_linear, 0.0, current_angular, last_dt_s);

  compute_movement();
  current_linear = (current_linear_left + current_linear_right) / 2;                          // m/s
  current_angular = atan((current_linear_right - current_linear_left) / LR_WHEELS_DISTANCE);  // rad/s
  // approx for low speeds
  //MobilityTracked::current_angular = (current_linear_right - current_linear_left) / LR_WHEELS_DISTANCE;  // rad/s

  unsigned long now = millis();
  if (((now - set_control_time_ms) > STOP_TIMEOUT_MS)) {
    // sabertooth watchdog must have stopped the motor, set as stopped and skip
    MobilityTracked::stop();
    return;
  }

  float advance_power = pid_advance.compute(target_linear, current_linear);
  float turn_power = pid_turn.compute(target_angular, current_angular);
  
  /*
  D_print("pid: ");
  D_print(current_linear);
  D_print("@");
  D_print((int8_t)advance_power);
  D_print(" ");
  D_print(current_angular);
  D_print("@");
  D_println((int8_t)turn_power);
  */
  
  sabertooth.drive((int8_t)advance_power);
  sabertooth.turn((int8_t)turn_power);
}

void MobilityTracked::stop() {
  tracked_set_target_velocities(0.0, 0.0);
  pid_advance.reset_errors();  //TODO could go inside tracked_set_target_velocities()
  pid_turn.reset_errors();     //TODO could go inside tracked_set_target_velocities()
  sabertooth.stop();
}

// TODO
void MobilityTracked::set_motor_enable(boolean enable) {
}

static void cmd_vel_cb(const void* cmd_vel) {
  set_target_velocities(msg_cmd_vel);

  //const geometry_msgs__msg__Twist* msg = (const geometry_msgs__msg__Twist*)msgin;
  //D_println(msg->linear.x);

  D_print("cmd_vel: ");
  D_print(msg_cmd_vel.linear.x);
  D_print(" ");
  D_println(msg_cmd_vel.angular.z);
}

static void joy_cb(const void* joy) {
  set_target_joy(msg_joy);
}

bool MobilityTracked::setup() {
  D_println("setup: mobility_tracked");
  
  if (!odom.setup()) {
    D_println("failure initializing odom_helper.");
    return false;
  }

  /* Initialize the motor driver */
  D_print("setup: sabertooth... ");
  SABERTOOTH_SERIAL.begin(9600, SERIAL_8N1, -1, SABERTOOTH_TX_PIN);
  sabertooth.drive(0);
  sabertooth.turn(0);
  sabertooth.setTimeout(STOP_TIMEOUT_MS);
  D_println("done.");

  //ESP32Encoder::useInternalWeakPullResistors = puType::down;
  // Enable the weak pull up resistors
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  
  
  //attachSingleEdge
  //attachFullQuad
  D_print("setup: encoders... ");
  encoder_rr_lft.attachSingleEdge(ENCODER_rr_lft_PIN_A, ENCODER_rr_lft_PIN_B);
  encoder_fr_lft.attachSingleEdge(ENCODER_fr_lft_PIN_A, ENCODER_fr_lft_PIN_B);
  encoder_fr_rgt.attachSingleEdge(ENCODER_fr_rgt_PIN_A, ENCODER_fr_rgt_PIN_B);
  encoder_rr_rgt.attachSingleEdge(ENCODER_rr_rgt_PIN_A, ENCODER_rr_rgt_PIN_B);
  encoder_rr_lft.clearCount();
  encoder_fr_lft.clearCount();
  encoder_fr_rgt.clearCount();
  encoder_rr_rgt.clearCount();
  last_encoder_us = micros();
  D_println("done.");

  /*
  D_print("MAX_WHEEL_ANGULAR ");D_println((float)MAX_WHEEL_ANGULAR);
  D_print("MAX_SPEED ");D_println((float)MAX_SPEED);
  D_print("MAX_TURNSPEED ");D_println((float)MAX_TURNSPEED);
  D_print("PID_LINEAL_KF ");D_println(PID_LINEAL_KF);
  D_print("PID_TURN_KF ");D_println((float)PID_TURN_KF);
  */

  sdescriptor_cmd_vel.type_support =
    (rosidl_message_type_support_t*)ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist);
  sdescriptor_cmd_vel.topic_name = TRACKED_TOPIC_CMD_VEL;
  sdescriptor_cmd_vel.msg = &msg_cmd_vel;
  sdescriptor_cmd_vel.callback = &cmd_vel_cb;
  micro_rosso::subscribers.push_back(&sdescriptor_cmd_vel);

  sdescriptor_joy.type_support =
    (rosidl_message_type_support_t*)ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy);
  sdescriptor_joy.topic_name = TRACKED_TOPIC_JOY;
  sdescriptor_joy.msg = &msg_joy;
  sdescriptor_joy.callback = &joy_cb;
  micro_rosso::subscribers.push_back(&sdescriptor_joy);

  micro_rosso::timer_control.callbacks.push_back(&control_cb);

  return true;
}

#endif // BUILD_MOBILITY_TRACKED
