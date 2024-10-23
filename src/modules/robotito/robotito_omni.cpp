#include "../build_modules.h"
#ifdef BUILD_ROBOTITO_OMNI

#include "robotito_omni.h"

#define NMOTORS 3
//static const uint8_t motor_pins[] = {27,26,37,39, 23,18,35,34, 25,33,36,38};  // v>=6
static const uint8_t motor_pins[] = {27,26,37,39, 23,18,35,34, 33,25,36,38};    // v<6

static const float ROBOT_R = 0.0675; //m
static const float ROBOT_R_3 = 1.0/(ROBOT_R*3.0);

static const float WHEEL_DIAMETER = 0.038; //m
static const float WHEEL_RADIUS = WHEEL_DIAMETER/2.0;

static const float ENC_CPR = 3;
static const float MOTOR_REDUCTION = 50;

static const int64_t MIN_ENC_ODOM_UPDATE = 10; // tics needed to update odomentry

static const float M_PI_6 = M_PI/6.0;
static const float M_PI_3 = M_PI/3.0;
static const float M_2_3 = 2.0/3.0;

static const float TICS_PER_REVOLUTION = ENC_CPR*MOTOR_REDUCTION;
static const float RAD_PER_TICK = 2.0*M_PI / TICS_PER_REVOLUTION;

static const float M_SEC_TO_TICS_SEC = 1/(RAD_PER_TICK * WHEEL_RADIUS);

static const float MAX_SPEED_POWER = 90; // power % at which MAX_SPEED_TICS is obtained
static const float MAX_SPEED_TICS = 1080; // tics/s at MAX_SPEED_POWER

static const float MAX_SPEED_RAD = MAX_SPEED_TICS * RAD_PER_TICK; // rad/s
static const float MAX_SPEED_LIN = MAX_SPEED_RAD * WHEEL_RADIUS; // m/s

static const float KF = MAX_SPEED_POWER / MAX_SPEED_TICS;
static const float KP = 0.01; //0.01; 
static const float KI = 0.05; //0.05;
static const float KD = 0.0;

#include "../vector_math.h"
#include <sensor_msgs/msg/joy.h>
#include <geometry_msgs/msg/twist.h>

static geometry_msgs__msg__Twist msg_cmd_vel;
static sensor_msgs__msg__Joy msg_joy;
static subscriber_descriptor sdescriptor_cmd_vel;
static subscriber_descriptor sdescriptor_joy;

#include "Cdrv8833.h"
#include <ESP32Encoder.h>
#include "../pid.h"
#include "../odom_helper.h"

unsigned long last_t_ms = 0;
unsigned long last_odom_ms = 0;

typedef struct {
	Cdrv8833 driver;
  ESP32Encoder encoder;
  
  float target_v_t_s; // tics_sec
  
  Pid pid;  
  float output;
  
  int64_t counter;
  int64_t last_count;
  //float current_v_t_s;
  
  int64_t last_odom_count;
} servo_t;

class DisableInterruptsGuard {
public:
  DisableInterruptsGuard() {
    noInterrupts();
  }

  ~DisableInterruptsGuard() {
    interrupts();
  }
};

static servo_t motors[NMOTORS];
//static odom_t odometry_position;
//static odom_t odometry_velocities;
static OdomHelper odom;


static vec3_t getW(float x_dot, float y_dot, float w_dot, float phi_r){
	vec3_t v = vec3(x_dot, y_dot, w_dot);

	mat3_t M = mat3(
		-sin(phi_r),         cos(phi_r),        ROBOT_R,
		-sin(M_PI_3 - phi_r), -cos(M_PI_3 - phi_r), ROBOT_R,
		 sin(M_PI_3 + phi_r), -cos(M_PI_3 + phi_r), ROBOT_R
	);

	vec3_t w = mat3_mul_vec3(M,v);
	return w;
}

static vec3_t getInverseW(float w_1, float w_2, float w_3, float phi){
	vec3_t x = vec3(w_1 *WHEEL_RADIUS, w_2 *WHEEL_RADIUS, w_3 *WHEEL_RADIUS);


	mat3_t A = mat3(
        -sin(phi)*M_2_3 , -cos(M_PI_6 + phi)*M_2_3, cos(phi - M_PI_6)*M_2_3,
		    cos(phi)*M_2_3  , -sin(M_PI_6 + phi)*M_2_3, sin(phi - M_PI_6)*M_2_3,
        ROBOT_R_3       , ROBOT_R_3               , ROBOT_R_3
	);

	vec3_t u = mat3_mul_vec3(A,x);
	return u;
}

RobotitoOmni::RobotitoOmni(){
};

static void control_cb(int64_t last_call_time) {
  //forward kinematics
  unsigned long now;
  {
    volatile DisableInterruptsGuard interrupt;
    int64_t count;
    now = millis();
    
    // protect against call bursts
    if (now-last_t_ms<TIMER_CONTROL_MS/4) {
      return;
    }
    
    for (int i=0; i<NMOTORS; i++) {
      count = motors[i].encoder.getCount();
      motors[i].last_count = count - motors[i].counter;
      motors[i].counter = count;    
    }
  }
  float last_dt_s = (float)(now - last_t_ms) / 1000.0;
  
  last_t_ms = now;
  
  for (int i=0; i<NMOTORS; i++) { 
    float current_v_t_s = motors[i].last_count / last_dt_s;
    float output = motors[i].pid.compute(motors[i].target_v_t_s, current_v_t_s);
    motors[i].driver.move((int8_t)output);   
    motors[i].output = output;
    //motors[i].current_v_t_s = current_v_t_s;
  }
  
  //reverse kinematics
  int64_t odom_increment0 = motors[0].counter-motors[0].last_odom_count;
  int64_t odom_increment1 = motors[1].counter-motors[1].last_odom_count;
  int64_t odom_increment2 = motors[2].counter-motors[2].last_odom_count;
  if (abs(odom_increment0) > MIN_ENC_ODOM_UPDATE ||
      abs(odom_increment1) > MIN_ENC_ODOM_UPDATE ||
      abs(odom_increment2) > MIN_ENC_ODOM_UPDATE) 
  {
    float odom_dt_s = (float)(now - last_odom_ms) / 1000.0;
    vec3_t odom_vels = getInverseW(
      odom_increment0 * RAD_PER_TICK / odom_dt_s, 
      odom_increment1 * RAD_PER_TICK / odom_dt_s, 
      odom_increment2 * RAD_PER_TICK / odom_dt_s, 
      odom.pos.phi
    );    
    odom.update_pos(odom_vels[0], odom_vels[1], odom_vels[2], odom_dt_s);
    last_odom_ms = now;
    motors[0].last_odom_count = motors[0].counter;
    motors[1].last_odom_count = motors[1].counter;
    motors[2].last_odom_count = motors[2].counter;
  }
  
 
  /*
  if (isnan(odom_vels[0]) || isnan(odom_vels[0]) || isnan(odom_vels[0])) {
    char nan_alarm[40] {0};
    sprintf(nan_alarm, "NAN! %ld %.4f %.4f %.4f", 
      last_t_ms, odom_vels[0], odom_vels[1], odom_vels[1]);
    micro_rosso::logger.log(nan_alarm);
  }
  if (odom_vels[0]>0.2 || odom_vels[1]>0.2 || odom_vels[2]>0.2 ) {
    char v_alarm[40] {0};
    sprintf(v_alarm, "V1! %ld %.4f %.4f %.4f %.4f", 
      last_t_ms, last_dt_s, odom_vels[0], odom_vels[1], odom_vels[1]);
    micro_rosso::logger.log(v_alarm);
    sprintf(v_alarm, "V2! %4f %.4f %.4f", 
      motors[0].last_count * RAD_PER_TICK / last_dt_s,
      motors[1].last_count * RAD_PER_TICK / last_dt_s,
      motors[2].last_count * RAD_PER_TICK / last_dt_s);
    micro_rosso::logger.log(v_alarm);

  }
  */
}

static void cmd_vel_cb(const void* cmd_vel) {
  //set_target_velocities(msg_cmd_vel);
  
  float x_dot = msg_cmd_vel.linear.x;
  float y_dot = msg_cmd_vel.linear.y;
  float w_dot = msg_cmd_vel.angular.z;
  float phi = 0.0; //TODO luaL_optnumber( L, 4, 0.0 );
  
  char cmd_vel_str[40] {0};
  sprintf(cmd_vel_str, "cmd_vel %.2f %.2f %.2f", x_dot, y_dot, w_dot);
  micro_rosso::logger.log(cmd_vel_str);

  vec3_t w = getW(x_dot, y_dot, w_dot, phi);

  sprintf(cmd_vel_str, "    out %.2f %.2f %.2f", w[0], w[1], w[2]);
  micro_rosso::logger.log(cmd_vel_str);

  for (int i=0; i<NMOTORS; i++) {
    motors[i].target_v_t_s = w[i] * M_SEC_TO_TICS_SEC;
    motors[i].pid.reset_errors();
  }
}

static void joy_cb(const void* joy) {
  //set_target_joy(msg_joy);
}

/*
static void report_cb() {
  char odom_str[100] {0};
  sprintf(odom_str, "> x=%.2f y=%.2f phi=%.2f", 
    odom.pos.x, odom.pos.y, odom.pos.phi);
  micro_rosso::logger.log(odom_str);
}
*/

static void ros_state_cb(ros_states state) {
  switch (state) {
    case WAITING_AGENT:
      break;
    case AGENT_AVAILABLE:
      break;
    case AGENT_CONNECTED:
      last_t_ms = millis();
      last_odom_ms = last_t_ms;
      for (int i=0; i<NMOTORS; i++) {
        motors[i].pid.reset_errors();
        motors[i].encoder.clearCount();
        motors[i].target_v_t_s = 0.0;
        motors[i].counter = 0;
        motors[i].last_count = 0;
        motors[i].last_odom_count = 0;
      }
      odom.reset();
      break;
    case AGENT_DISCONNECTED:
      for (int i=0; i<NMOTORS; i++) {
        motors[i].driver.stop();
      }  
      break;
    default:
      break;
  }
}

static bool init_motors() {
  // Enable the weak pull up resistors
  ESP32Encoder::useInternalWeakPullResistors = puType::up;

  for (int i=0; i<NMOTORS; i++) {
    int8_t pin1 = motor_pins[4*i];
    int8_t pin2 = motor_pins[4*i+1];
    int8_t encA = motor_pins[4*i+2];
    int8_t encB = motor_pins[4*i+3];

    char setting_str[54] {0};
    sprintf(setting_str, "omni Setting motor %d pins:%d,%d enc:%d,%d... ", 
      i, pin1, pin2, encA, encB);
    D_print(setting_str);

    //driver
    motors[i].driver.init(pin1, pin2, i, false);
    motors[i].driver.setFrequency(20000);
    motors[i].driver.setDecayMode(drv8833DecaySlow);

    //encoder
    motors[i].encoder.attachSingleEdge(encA, encB);
    motors[i].encoder.clearCount();
    
    //pid
    motors[i].pid.update_constants(
      -MAX_SPEED_POWER, MAX_SPEED_POWER, KF, KP, KI, KD);
    motors[i].target_v_t_s=0;
    
    D_println(" done.");
  }

  last_t_ms = millis();
  
  odom.reset();

  return true;
}

bool RobotitoOmni::setup() {
  D_println("setup mobility: robotito");
  
  if (!init_motors()) {
    D_println("failure initializing motors.");
    return false;
  }
  
  if (!odom.setup()) {
    D_println("failure initializing odom_helper.");
    return false;
  }

  sdescriptor_cmd_vel.type_support =
    (rosidl_message_type_support_t*)ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist);
  sdescriptor_cmd_vel.topic_name = ROBOTITO_TOPIC_CMD_VEL;
  sdescriptor_cmd_vel.msg = &msg_cmd_vel;
  sdescriptor_cmd_vel.callback = &cmd_vel_cb;
  micro_rosso::subscribers.push_back(&sdescriptor_cmd_vel);

  sdescriptor_joy.type_support =
    (rosidl_message_type_support_t*)ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy);
  sdescriptor_joy.topic_name = ROBOTITO_TOPIC_JOY;
  sdescriptor_joy.msg = &msg_joy;
  sdescriptor_joy.callback = &joy_cb;
  micro_rosso::subscribers.push_back(&sdescriptor_joy);

  micro_rosso::timer_control.callbacks.push_back(&control_cb);
  
  //micro_rosso::timer_report.callbacks.push_back(&report_cb);
  
  micro_rosso::ros_state_listeners.push_back(ros_state_cb);

  return true;
}


#endif // BUILD_ROBOTITO_OMNI

