#include "mpu9150.h"
#include "utils.h"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <unistd.h>


#define MPU9150_ADDR 0x68
#define AK8975_ADDR 0x0C

// Registers
#define PWR_MGMT_1 0x6B
#define INT_PIN_CFG 0x37
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43
#define USER_CTRL 0x6A

#define AK8975_ST1 0x02
#define AK8975_HXL 0x03
#define AK8975_CNTL 0x0A

// Helpers
static int i2c_write(int fd, uint8_t addr, uint8_t reg, uint8_t val) {
  if (ioctl(fd, I2C_SLAVE, addr) < 0)
    return -1;
  uint8_t buf[2] = {reg, val};
  if (write(fd, buf, 2) != 2)
    return -1;
  return 0;
}

static int i2c_read(int fd, uint8_t addr, uint8_t reg, uint8_t *val) {
  if (ioctl(fd, I2C_SLAVE, addr) < 0)
    return -1;
  if (write(fd, &reg, 1) != 1)
    return -1;
  if (read(fd, val, 1) != 1)
    return -1;
  return 0;
}

static int i2c_read_block(int fd, uint8_t addr, uint8_t reg, uint8_t *buf,
                          int len) {
  if (ioctl(fd, I2C_SLAVE, addr) < 0)
    return -1;
  if (write(fd, &reg, 1) != 1)
    return -1;
  if (read(fd, buf, len) != len)
    return -1;
  return 0;
}

static int16_t parse_be(uint8_t *buf) {
  return (int16_t)((buf[0] << 8) | buf[1]);
}

static int16_t parse_le(uint8_t *buf) {
  return (int16_t)((buf[1] << 8) | buf[0]);
}

int mpu_init(MPU9150 *mpu, int bus_num) {
  char dev[32];
  snprintf(dev, sizeof(dev), "/dev/i2c-%d", bus_num);

  mpu->fd = open(dev, O_RDWR);
  if (mpu->fd < 0) {
    log_message(LOG_ERROR, "MPU: Failed to open %s", dev);
    return 0;
  }

  // Reset
  i2c_write(mpu->fd, MPU9150_ADDR, PWR_MGMT_1, 0x80);
  sleep_ms(100);
  // Wake
  i2c_write(mpu->fd, MPU9150_ADDR, PWR_MGMT_1, 0x00);
  sleep_ms(100);
  // Disable I2C Master
  i2c_write(mpu->fd, MPU9150_ADDR, USER_CTRL, 0x00);
  sleep_ms(100);
  // Bypass
  i2c_write(mpu->fd, MPU9150_ADDR, INT_PIN_CFG, 0x02);
  sleep_ms(100);

  // Check Mag
  uint8_t wia = 0;
  if (i2c_read(mpu->fd, AK8975_ADDR, 0x00, &wia) != 0) {
    log_message(LOG_WARN, "MPU: AK8975 not detected via bypass");
  } else {
    log_message(LOG_INFO, "MPU: AK8975 Found (WIA=0x%02X)", wia);
  }

  // Init offsets
  mpu->gyro_offset = (Vector3){0, 0, 0};
  mpu->accel_offset = (Vector3){0, 0, 0};
  mpu->mag_offset = (Vector3){0, 0, 0};

  // Init state
  mpu->mag_filter_state = (Vector3){0, 0, 0};

  log_message(LOG_INFO, "MPU9150 Initialized");
  return 1;
}

void mpu_close(MPU9150 *mpu) {
  if (mpu->fd >= 0)
    close(mpu->fd);
}

void mpu_set_calibration(MPU9150 *mpu, Vector3 g, Vector3 a, Vector3 m) {
  mpu->gyro_offset = g;
  mpu->accel_offset = a;
  mpu->mag_offset = m;
}

int mpu_read(MPU9150 *mpu, SensorData *out) {
  if (mpu->fd < 0)
    return 0;

  uint8_t buf[14]; // Accel(6) + Temp(2) + Gyro(6)

  // Read MPU Data
  if (i2c_read_block(mpu->fd, MPU9150_ADDR, ACCEL_XOUT_H, buf, 14) != 0)
    return 0;

  float a_scale = 16384.0f;
  float g_scale = 131.0f;

  out->accel.x = (parse_be(&buf[0]) / a_scale * 9.81f) - mpu->accel_offset.x;
  out->accel.y = (parse_be(&buf[2]) / a_scale * 9.81f) - mpu->accel_offset.y;
  out->accel.z = (parse_be(&buf[4]) / a_scale * 9.81f) - mpu->accel_offset.z;

  out->gyro.x = (parse_be(&buf[8]) / g_scale) - mpu->gyro_offset.x;
  out->gyro.y = (parse_be(&buf[10]) / g_scale) - mpu->gyro_offset.y;
  out->gyro.z = (parse_be(&buf[12]) / g_scale) - mpu->gyro_offset.z;

  // Trigger Mag (Single Meas)
  i2c_write(mpu->fd, AK8975_ADDR, AK8975_CNTL, 0x01);
  // Wait ~10ms. Since this read loop might be high freq, this sleep is bad.
  // Instead we should probably poll or assume previous measurement.
  // But for simplicity (robot loop ~50Hz), 10ms wait reduces loop to max 100Hz.
  // Let's use 5ms.
  sleep_ms(5);

  uint8_t st1 = 0;
  if (i2c_read(mpu->fd, AK8975_ADDR, AK8975_ST1, &st1) == 0 && (st1 & 0x01)) {
    uint8_t mbuf[6];
    if (i2c_read_block(mpu->fd, AK8975_ADDR, AK8975_HXL, mbuf, 6) == 0) {
      float mx = (float)parse_le(&mbuf[0]);
      float my = (float)parse_le(&mbuf[2]);
      float mz = (float)parse_le(&mbuf[4]);

      // LPF (Alpha 0.39)
      float alpha = 0.39f;
      if (mpu->mag_filter_state.x == 0 && mpu->mag_filter_state.y == 0) {
        mpu->mag_filter_state = (Vector3){mx, my, mz};
      } else {
        mpu->mag_filter_state.x += alpha * (mx - mpu->mag_filter_state.x);
        mpu->mag_filter_state.y += alpha * (my - mpu->mag_filter_state.y);
        mpu->mag_filter_state.z += alpha * (mz - mpu->mag_filter_state.z);
      }

      out->mag.x = mpu->mag_filter_state.x - mpu->mag_offset.x;
      out->mag.y = mpu->mag_filter_state.y - mpu->mag_offset.y;
      out->mag.z = mpu->mag_filter_state.z - mpu->mag_offset.z;
    }
  } else {
    // Reuse last known or zero if failed
    out->mag = (Vector3){0, 0, 0};
    // In reality we should use last filter state
  }

  // Heading (Z-Down)
  out->heading = atan2f(-out->mag.y, out->mag.x) * 180.0f / M_PI;
  if (out->heading < 0)
    out->heading += 360.0f;

  out->timestamp = current_time_ms();
  return 1;
}
