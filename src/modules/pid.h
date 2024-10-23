#ifndef __pid_h
#define __pid_h

class Pid {

public:
  Pid();
  Pid(float min_out, float max_out, float kf, float kp, float ki, float kd);
  float compute(float setpoint, float measured_value);
  void update_constants(float min_out, float max_out, float kf, float kp, float ki, float kd);
  void reset_errors();

private:
  float min_out;
  float max_out;
  float kf;  // forward controller
  float kp;
  float ki;
  float kd;
  float integral;
  float prev_error;
};

#endif  // __pid_h
