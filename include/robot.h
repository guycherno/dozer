#ifndef ROBOT_H
#define ROBOT_H

#include "motor.h"
#include "mpu9150.h"
#include "pid.h"

typedef enum { ROBOT_IDLE, ROBOT_MOVING, ROBOT_TURNING } RobotState;

typedef struct {
  // Subsystems
  MotorDriver motor;
  MPU9150 mpu;

  // PIDs
  PID pid_heading;
  PID pid_turn;

  // Config
  float speed_factor_forward;
  float speed_factor_backward;
  float turn_factor_cw;
  float turn_factor_ccw;

  // State
  RobotState state;
  float current_heading;
  float target_heading;
  float accumulated_turn;

  // Move State
  float move_speed;
  float move_target_dist;
  long long move_start_time;

  // Turn State
  float turn_target_delta; // Desired change
  int turn_direction;      // 1 = CW (Right), -1 = CCW (Left)

  // Data
  SensorData last_data;

} Robot;

int robot_init(Robot *robot);
void robot_close(Robot *robot);
void robot_update(Robot *robot, float dt);
void robot_stop(Robot *robot);

// Command Interface
// Returns 1 on success, 0 on failure. Fills out_msg.
int robot_execute_command(Robot *robot, const char *cmd, char *out_msg,
                          int max_len);

// Status
void robot_get_status_json(Robot *robot, char *out_json, int max_len);

#endif
