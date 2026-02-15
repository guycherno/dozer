#ifndef PID_H
#define PID_H

typedef struct {
  float kp;
  float ki;
  float kd;
  float min_output;
  float max_output;
  float integral_limit;

  // State
  float integral;
  float prev_error;
  long long last_time;
} PID;

void pid_init(PID *pid, float kp, float ki, float kd, float min_out,
              float max_out);
void pid_reset(PID *pid);
float pid_update(PID *pid, float error, float dt);

#endif
