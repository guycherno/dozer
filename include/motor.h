#ifndef MOTOR_H
#define MOTOR_H

typedef struct {
  char port[64];
  int baudrate;
  int fd; // File descriptor
} MotorDriver;

int motor_init(MotorDriver *motor, const char *port, int baudrate);
int motor_open(MotorDriver *motor);
void motor_close(MotorDriver *motor);
void motor_set_speed(MotorDriver *motor, float left, float right);
void motor_stop(MotorDriver *motor);

#endif
