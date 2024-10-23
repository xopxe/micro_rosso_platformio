#include "odom_helper.h"
odom_t OdomHelper::pos;
odom_t OdomHelper::vel;

static nav_msgs__msg__Odometry msg_odom;
static publisher_descriptor pdescriptor_odom;

#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

#define RCNOCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    (void)temp_rc; \
  }

static void euler_to_quat(float roll, float pitch, float yaw, float *q) {
  float cy = cos(yaw * 0.5);
  float sy = sin(yaw * 0.5);
  float cp = cos(pitch * 0.5);
  float sp = sin(pitch * 0.5);
  float cr = cos(roll * 0.5);
  float sr = sin(roll * 0.5);

  q[0] = cy * cp * cr + sy * sp * sr;
  q[1] = cy * cp * sr - sy * sp * cr;
  q[2] = sy * cp * sr + cy * sp * cr;
  q[3] = sy * cp * cr - cy * sp * sr;
}

OdomHelper::OdomHelper() {
  msg_odom.header.frame_id = micro_ros_string_utilities_set(msg_odom.header.frame_id, "odom");
  msg_odom.child_frame_id = micro_ros_string_utilities_set(msg_odom.child_frame_id, "base_footprint");

/*  msg_odom.twist.twist.linear.y = 0.0;
  msg_odom.twist.twist.linear.z = 0.0;
  msg_odom.twist.twist.angular.x = 0.0;
  msg_odom.twist.twist.angular.y = 0.0;
  msg_odom.pose.pose.position.z = 0.0; */
  
  // we don't change these anymore
  msg_odom.pose.covariance[0] = 0.0001;
  msg_odom.pose.covariance[7] = 0.0001;
  msg_odom.pose.covariance[35] = 0.0001;
  
  msg_odom.twist.covariance[0] = 0.0001;
  msg_odom.twist.covariance[7] = 0.0001;
  msg_odom.twist.covariance[35] = 0.0001;
}

void OdomHelper::set( float x, float y, float phi, float vx, float vy, float vphi ) {
  OdomHelper::pos.x = x;
  OdomHelper::pos.y = y;
  OdomHelper::pos.phi = phi;
  OdomHelper::vel.x = vx;
  OdomHelper::vel.y = vy;
  OdomHelper::vel.phi = vphi;
}

void OdomHelper::reset() {
  set(0.0,0.0,0.0,0.0,0.0,0.0);
}


void OdomHelper::update_pos( float vx, float vy, float vphi, float dt ) {
  OdomHelper::vel.x = vx;
  OdomHelper::vel.y = vy;
  OdomHelper::vel.phi = vphi;
  
  OdomHelper::pos.x += vx * dt;
  OdomHelper::pos.y += vy * dt;
  OdomHelper::pos.phi += vphi * dt;
}

static void report_cb(int64_t last_call_time) {

  //robot's position in x,y,phi
  msg_odom.pose.pose.position.x = OdomHelper::pos.x;
  msg_odom.pose.pose.position.y = OdomHelper::pos.y;
  //msg_odom.pose.pose.position.z = 0;
  
  //calculate robot's heading in quaternion angle
  //ROS has a function to calculate yaw in quaternion angle
  float q[4];
  euler_to_quat(0, 0, OdomHelper::pos.phi, q);
  //robot's heading in quaternion
  msg_odom.pose.pose.orientation.x = q[1];
  msg_odom.pose.pose.orientation.y = q[2];
  msg_odom.pose.pose.orientation.z = q[3];
  msg_odom.pose.pose.orientation.w = q[0];

  //speed from encoders
  msg_odom.twist.twist.linear.x = OdomHelper::vel.x;
  msg_odom.twist.twist.linear.y = OdomHelper::vel.y;
  msg_odom.twist.twist.angular.z = OdomHelper::vel.phi;
  
  micro_rosso::set_timestamp(msg_odom.header.stamp); //TODO replace with real timestamp
  RCNOCHECK(rcl_publish(&pdescriptor_odom.publisher, &msg_odom, NULL));
  

D_print(OdomHelper::vel.x);
D_print(" O ");
D_println(OdomHelper::vel.phi);


}


bool OdomHelper::setup() {
  D_print("setup: odom_helper... ");

  set(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  pdescriptor_odom.qos = QOS_DEFAULT;
  pdescriptor_odom.type_support = (rosidl_message_type_support_t *)
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry);
  pdescriptor_odom.topic_name = ODOM_TOPIC_ODOM;
  micro_rosso::publishers.push_back(&pdescriptor_odom);

  micro_rosso::timer_report.callbacks.push_back(&report_cb);

  D_println("done.");
  return true;
}

