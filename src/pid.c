#include "pid.h"
#include <stddef.h>

void pid_init(PID *pid, float kp, float ki, float kd, float min_out,
              float max_out) {
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->min_output = min_out;
  pid->max_output = max_out;
  pid->integral_limit = 0.0f; // Default no limit or set manually
  pid_reset(pid);
}

void pid_reset(PID *pid) {
  pid->integral = 0.0f;
  pid->prev_error = 0.0f;
  pid->last_time = 0;
}

float pid_update(PID *pid, float error, float dt) {
  if (dt <= 0.0f)
    return 0.0f;

  // P
  float p_term = pid->kp * error;

  // I
  pid->integral += error * dt;
  // TODO: limit integral if needed
  float i_term = pid->ki * pid->integral;

  // D
  float derivative = (error - pid->prev_error) / dt;
  float d_term = pid->kd * derivative;

  pid->prev_error = error;

  float output = p_term + i_term + d_term;
  float final_output = output;

  // Friction Compensation
  if (pid->min_output > 0) {
    if (output > 0) {
      final_output += pid->min_output;
    } else if (output < 0) {
      final_output -= pid->min_output;
    }
  }

  // Clip
  if (pid->max_output > 0) {
    if (final_output > pid->max_output)
      final_output = pid->max_output;
    if (final_output < -pid->max_output)
      final_output = -pid->max_output;
  }

  return final_output;
}
