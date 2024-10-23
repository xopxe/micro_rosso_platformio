// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "pid.h"

Pid::Pid() {
}

Pid::Pid(float min_out, float max_out, float kf, float kp, float ki, float kd)
  : min_out(min_out),
    max_out(max_out),
    kf(kf),
    kp(kp),
    ki(ki),
    kd(kd) {
}

float Pid::compute(float setpoint, float measured_value) {
  float pid;

  //setpoint is constrained between min and max to prevent pid from having too much error
  if (setpoint > max_out) {
    setpoint = max_out;
  } else if (setpoint < min_out) {
    setpoint = min_out;
  }

  float error = setpoint - measured_value;
  float new_integral = integral + error;
  float derivative = error - prev_error;

/*
  if (setpoint == 0 && error == 0) {
    integral = 0;
    derivative = 0;
  }
*/

  pid = kf * setpoint;
  pid += kp * error;
  pid += ki * new_integral;
  pid += kd * derivative;

  if (pid > max_out) {
    pid = max_out;
  } else if (pid < min_out) {
    pid = min_out;
  } else {
    integral = new_integral;  // only here, for windup protection
  }

  prev_error = error;

  return pid;
}

void Pid::update_constants(float min_out, float max_out, float kf, float kp, float ki, float kd) {
  Pid::min_out = min_out;
  Pid::max_out = max_out;
  Pid::kf = kf;
  Pid::kp = kp;
  Pid::ki = ki;
  Pid::kd = kd;
}

void Pid::reset_errors() {
  integral = 0;
  prev_error = 0;
}
