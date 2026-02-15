#ifndef MPU9150_H
#define MPU9150_H

#include <stdint.h>

typedef struct {
  float x, y, z;
} Vector3;

typedef struct {
  Vector3 accel;
  Vector3 gyro;
  Vector3 mag;
  float heading;
  long long timestamp;
} SensorData;

typedef struct {
  int fd; // I2C file descriptor
  int address;

  // Calibration
  Vector3 gyro_offset;
  Vector3 accel_offset;
  Vector3 mag_offset;

  // State
  Vector3 mag_filter_state;
  // float lpf_alpha; // Fixed at 0.39 for now
} MPU9150;

int mpu_init(MPU9150 *mpu, int bus_num);
void mpu_close(MPU9150 *mpu);
int mpu_read(MPU9150 *mpu, SensorData *out_data);

// Calibration helpers (load/save done in robot.c via json)
void mpu_set_calibration(MPU9150 *mpu, Vector3 g_off, Vector3 a_off,
                         Vector3 m_off);

#endif
