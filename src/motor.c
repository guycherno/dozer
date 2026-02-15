#include "motor.h"
#include "utils.h"

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

int motor_open(MotorDriver *motor) {
  if (motor->fd != -1)
    return 1; // Already open

  motor->fd = open(motor->port, O_RDWR | O_NOCTTY | O_NDELAY);
  if (motor->fd == -1) {
    log_message(LOG_ERROR, "Motor: Unable to open port %s", motor->port);
    return 0;
  }

  struct termios options;
  tcgetattr(motor->fd, &options);

  // Set Baudrate (assuming B115200 for now, handle others if needed)
  cfsetispeed(&options, B115200);
  cfsetospeed(&options, B115200);

  // 8N1
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;

  // No flow control
  options.c_cflag &= ~CRTSCTS;

  // Enable receiver, local mode
  options.c_cflag |= (CLOCAL | CREAD);

  // Raw input/output
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_oflag &= ~OPOST;

  tcsetattr(motor->fd, TCSANOW, &options);

  // Clear flags
  fcntl(motor->fd, F_SETFL, 0);

  log_message(LOG_INFO, "Motor: Connected on %s", motor->port);
  return 1;
}

int motor_init(MotorDriver *motor, const char *port, int baudrate) {
  strncpy(motor->port, port, sizeof(motor->port) - 1);
  motor->baudrate = baudrate;
  motor->fd = -1;
  return motor_open(motor);
}

void motor_close(MotorDriver *motor) {
  if (motor->fd != -1) {
    tcdrain(motor->fd);
    usleep(50000); // 50ms safety delay
    close(motor->fd);
    motor->fd = -1;
  }
}

static void send_cmd(MotorDriver *motor, const char *cmd) {
  if (motor->fd == -1)
    return;
  char buf[64];
  // Add ! and \n
  snprintf(buf, sizeof(buf), "!%s\n", cmd);
  write(motor->fd, buf, strlen(buf));
  // log_message(LOG_DEBUG, "Motor Send: %s", buf);
}

void motor_set_speed(MotorDriver *motor, float left, float right) {
  // Clamp
  if (left > 100)
    left = 100;
  if (left < -100)
    left = -100;
  if (right > 100)
    right = 100;
  if (right < -100)
    right = -100;

  char cmd[32];

  // Left
  if (left > 0)
    snprintf(cmd, sizeof(cmd), "left-motor-forward %d", (int)left);
  else if (left < 0)
    snprintf(cmd, sizeof(cmd), "left-motor-reverse %d", (int)(-left));
  else
    snprintf(cmd, sizeof(cmd), "left-motor-stop");
  send_cmd(motor, cmd);

  // Right
  if (right > 0)
    snprintf(cmd, sizeof(cmd), "right-motor-forward %d", (int)right);
  else if (right < 0)
    snprintf(cmd, sizeof(cmd), "right-motor-reverse %d", (int)(-right));
  else
    snprintf(cmd, sizeof(cmd), "right-motor-stop");
  send_cmd(motor, cmd);
}

void motor_stop(MotorDriver *motor) {
  for (int i = 0; i < 3; i++) {
    send_cmd(motor, "left-motor-stop");
    send_cmd(motor, "right-motor-stop");
    usleep(5000); // 5ms delay between bursts
  }
}
