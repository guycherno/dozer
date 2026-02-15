#include "robot.h"
#include "json.h"
#include "utils.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CALIBRATION_FILE "calibration.json"

static void load_calibration(Robot *robot) {
  FILE *f = fopen(CALIBRATION_FILE, "r");
  if (!f) {
    log_message(LOG_WARN, "Robot: Calibration file not found, using defaults.");
    return;
  }

  fseek(f, 0, SEEK_END);
  long fsize = ftell(f);
  fseek(f, 0, SEEK_SET);

  char *string = malloc(fsize + 1);
  fread(string, 1, fsize, f);
  fclose(f);
  string[fsize] = 0;

  // Parse
  float val;
  if (json_get_float(string, "speed_factor", &val)) {
    robot->speed_factor_forward = val;
    robot->speed_factor_backward = val;
  }
  if (json_get_float(string, "speed_factor_forward", &val))
    robot->speed_factor_forward = val;
  if (json_get_float(string, "speed_factor_backward", &val))
    robot->speed_factor_backward = val;
  if (json_get_float(string, "turn_factor_cw", &val))
    robot->turn_factor_cw = val;
  if (json_get_float(string, "turn_factor_ccw", &val))
    robot->turn_factor_ccw = val;

  // Heading PID
  float hkp = 1.0, hki = 0.0, hkd = 0.1;
  json_get_float(string, "heading_kp", &hkp);
  json_get_float(string, "heading_ki", &hki);
  json_get_float(string, "heading_kd", &hkd);
  pid_init(&robot->pid_heading, hkp, hki, hkd, 0.0, 20.0);

  // Turn PID
  float tkp = 2.0, tki = 0.0, tkd = 0.1, tmin = 35.0;
  json_get_float(string, "turn_kp_cw",
                 &tkp); // Assuming CW/CCW similar or picking one for init
  json_get_float(string, "turn_ki_cw", &tki);
  json_get_float(string, "turn_kd_cw", &tkd);
  json_get_float(string, "turn_min_output_cw", &tmin);
  pid_init(&robot->pid_turn, tkp, tki, tkd, tmin, 100.0);

  // MPU Offsets
  Vector3 g_off = {0}, a_off = {0}, m_off = {0};
  char obj_buf[256];

  if (json_get_object(string, "gyro", obj_buf, sizeof(obj_buf))) {
    json_get_float(obj_buf, "x", &g_off.x);
    json_get_float(obj_buf, "y", &g_off.y);
    json_get_float(obj_buf, "z", &g_off.z);
  }

  if (json_get_object(string, "accel", obj_buf, sizeof(obj_buf))) {
    json_get_float(obj_buf, "x", &a_off.x);
    json_get_float(obj_buf, "y", &a_off.y);
    json_get_float(obj_buf, "z", &a_off.z);
  }

  if (json_get_object(string, "mag", obj_buf, sizeof(obj_buf))) {
    json_get_float(obj_buf, "x", &m_off.x);
    json_get_float(obj_buf, "y", &m_off.y);
    json_get_float(obj_buf, "z", &m_off.z);
  }

  mpu_set_calibration(&robot->mpu, g_off, a_off, m_off);

  log_message(LOG_INFO, "Robot: Calibration loaded.");
  free(string);
}

int robot_init(Robot *robot) {
  memset(robot, 0, sizeof(Robot));

  // Defaults
  robot->speed_factor_forward = 0.002125f;
  robot->speed_factor_backward = 0.002125f;
  robot->turn_factor_cw = 1.0f;
  robot->turn_factor_ccw = 1.0f;

  // Init Drivers
  if (!motor_init(&robot->motor, "/dev/ttyAMA0", 115200)) {
    return 0; // Fatal?
  }

  if (!mpu_init(&robot->mpu, 1)) {
    log_message(LOG_WARN, "Robot: MPU init failed, continuing without sensor.");
  }

  // Init PIDs (defaults, overridden by load_calib)
  pid_init(&robot->pid_heading, 1.0, 0.0, 0.1, 0.0, 20.0);
  pid_init(&robot->pid_turn, 2.0, 0.0, 0.1, 35.0, 100.0);

  load_calibration(robot);
  motor_close(&robot->motor); // Start closed (UART off)

  robot->state = ROBOT_IDLE;
  return 1;
}

void robot_close(Robot *robot) {
  motor_stop(&robot->motor);
  motor_close(&robot->motor);
  mpu_close(&robot->mpu);
}

void robot_stop(Robot *robot) {
  robot->state = ROBOT_IDLE;
  motor_stop(&robot->motor);
  motor_close(&robot->motor); // Turn off UART
}

void robot_update(Robot *robot, float dt) {
  // Read Sensor
  mpu_read(&robot->mpu, &robot->last_data);
  robot->current_heading = robot->last_data.heading;

  if (robot->state == ROBOT_IDLE) {
    motor_stop(&robot->motor);
    return;
  }

  if (robot->state == ROBOT_MOVING) {
    long long now = current_time_ms();
    float elapsed = (now - robot->move_start_time) / 1000.0f;

    float factor = (robot->move_speed >= 0) ? robot->speed_factor_forward
                                            : robot->speed_factor_backward;
    float dist_covered = fabsf(robot->move_speed) * factor * elapsed;

    if (dist_covered >= robot->move_target_dist) {
      log_message(LOG_INFO, "Move Complete");
      robot_stop(robot);
      return;
    }

    // Heading Hold
    float error = robot->target_heading - robot->current_heading;
    while (error > 180)
      error -= 360;
    while (error < -180)
      error += 360;

    float correction = pid_update(&robot->pid_heading, error, dt);

    float ls = robot->move_speed + correction;
    float rs = robot->move_speed - correction;

    motor_set_speed(&robot->motor, ls, rs);
  } else if (robot->state == ROBOT_TURNING) {
    float gz = robot->last_data.gyro.z;
    float factor = (robot->turn_direction == 1) ? robot->turn_factor_cw
                                                : robot->turn_factor_ccw;
    robot->accumulated_turn += gz * factor * dt;

    float error = robot->turn_target_delta - robot->accumulated_turn;

    float output = pid_update(&robot->pid_turn, error, dt);

    // Check completion (2 deg, 15 deg/s) - using relaxed condition
    if (fabsf(error) < 2.0f && fabsf(gz) < 15.0f) {
      log_message(LOG_INFO, "Turn Complete");
      robot_stop(robot);
      return;
    }

    float ls = -output;
    float rs = output;
    motor_set_speed(&robot->motor, ls, rs);
  }
}

int robot_execute_command(Robot *robot, const char *cmd, char *out_msg,
                          int max_len) {
  // Basic parsing: "cmd arg1 arg2"
  char buffer[128];
  strncpy(buffer, cmd, sizeof(buffer) - 1);

  char *saveptr;
  char *token = strtok_r(buffer, " ", &saveptr);
  if (!token) {
    snprintf(out_msg, max_len, "Empty command");
    return 0;
  }

  // to lower?
  // Let's assume input is correct case or handle sensitively.
  // Python code used .lower(). C is case sensitive.
  // We should probably strcasecmp.

  if (strcasecmp(token, "stop") == 0) {
    robot_stop(robot);
    snprintf(out_msg, max_len, "Stopped");
    return 1;
  } else if (strcasecmp(token, "forward") == 0 ||
             strcasecmp(token, "backward") == 0) {
    char *speed_str = strtok_r(NULL, " ", &saveptr);
    char *dist_str = strtok_r(NULL, " ", &saveptr);

    if (!speed_str || !dist_str) {
      snprintf(out_msg, max_len, "Usage: %s <speed> <dist>", token);
      return 0;
    }

    float speed = strtof(speed_str, NULL);
    float dist = strtof(dist_str, NULL);

    if (strcasecmp(token, "backward") == 0)
      speed = -speed;

    if (!motor_open(&robot->motor)) {
      snprintf(out_msg, max_len, "Failed to open motor port");
      return 0;
    }

    robot->state = ROBOT_MOVING;
    robot->move_speed = speed;
    robot->move_target_dist = dist;
    robot->move_start_time = current_time_ms();
    robot->target_heading = robot->current_heading;
    pid_reset(&robot->pid_heading);

    float factor = (speed >= 0) ? robot->speed_factor_forward
                                : robot->speed_factor_backward;
    snprintf(out_msg, max_len, "Moving speed=%.1f dist=%.1f sf=%.6f", speed,
             dist, factor);
    return 1;
  } else if (strcasecmp(token, "turn") == 0) {
    char *dir_str = strtok_r(NULL, " ", &saveptr);
    char *deg_str = strtok_r(NULL, " ", &saveptr);

    if (!dir_str || !deg_str) {
      snprintf(out_msg, max_len, "Usage: Turn <left/right> <deg>");
      return 0;
    }

    float deg = strtof(deg_str, NULL);
    int direction = 1; // Right/CW
    if (strcasecmp(dir_str, "left") == 0)
      direction = -1; // Left/CCW
    else if (strcasecmp(dir_str, "right") != 0) {
      snprintf(out_msg, max_len, "Direction must be left or right");
      return 0;
    }

    if (!motor_open(&robot->motor)) {
      snprintf(out_msg, max_len, "Failed to open motor port");
      return 0;
    }

    robot->state = ROBOT_TURNING;
    robot->turn_direction = direction;
    robot->accumulated_turn = 0.0f;

    // Target Delta: Right(-), Left(+) -> Matches Python?
    // Python: Right = -degrees, Left = +degrees.
    if (direction == 1)
      robot->turn_target_delta = -deg; // Right
    else
      robot->turn_target_delta = deg; // Left

    // Reset PID with correct params (TODO: Load specific params from config)
    pid_reset(&robot->pid_turn);

    snprintf(out_msg, max_len, "Turning %s %.1f", dir_str, deg);
    return 1;
  } else if (strcasecmp(token, "reload") == 0) {
    load_calibration(robot);
    snprintf(out_msg, max_len, "Configuration Reloaded");
    return 1;
  }

  snprintf(out_msg, max_len, "Unknown command");
  return 0;
}

void robot_get_status_json(Robot *robot, char *out_json, int max_len) {
  // Minimal status
  char state_str[32];
  switch (robot->state) {
  case ROBOT_IDLE:
    strcpy(state_str, "IDLE");
    break;
  case ROBOT_MOVING:
    strcpy(state_str, "MOVING");
    break;
  case ROBOT_TURNING:
    strcpy(state_str, "TURNING");
    break;
  default:
    strcpy(state_str, "UNKNOWN");
    break;
  }

  // Manual JSON construction
  snprintf(out_json, max_len, "{\"state\":\"%s\",\"heading\":%.2f,\"logs\":[]}",
           state_str, robot->current_heading);
}
