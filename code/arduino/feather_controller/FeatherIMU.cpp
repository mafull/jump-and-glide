#include "imu.h"

#include <Wire.h>
#include <Arduino.h>


#define MPU6050_ADDRESS       0x68

#define MPU6050_WHO_AM_I      0x75

#define MPU6050_SMPRT_DIV     0x19
#define MPU6050_CONFIG        0x1A
#define MPU6050_GYRO_CONFIG   0x1B
#define MPU6050_ACCEL_CONFIG  0x1C

#define MPU6050_ACX           0x3B
#define MPU6050_GYX           0x43

#define MPU6050_PWR_MGMT_1    0x6B
#define MPU6050_PWR_MGMT_2    0x6C

#define GYRO_SENSITIVITY      131


IMU imu;


bool FeatherIMU::init() {
  Wire.begin();

  Wire.beginTransmission(MPU6050_ADDRESS);
  // PWR_MGMT_1 register
  // Set to 0 to wake MPU6050
  Wire.write(MPU6050_PWR_MGMT_1);
  Wire.write(0);
  Wire.endTransmission(true);

  int8_t whoAmI;
  Wire.beginTransmission(MPU6050_ADDRESS);
  // WHO_AM_I register
  // Returns address of device
  Wire.write(MPU6050_WHO_AM_I);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDRESS, 1, true);
  whoAmI = Wire.read();

  if (whoAmI != MPU6050_ADDRESS) return false;

  // Accelerometer update rate is 1kHz, so set sample rate to 1kHz
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(MPU6050_SMPRT_DIV);
  Wire.write(7);
  Wire.endTransmission(true);

  // Disable DLPF and FSYNC
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(MPU6050_CONFIG);
  Wire.write(7);
  Wire.endTransmission(true);

  // Set gyro range to +-250deg/s
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(MPU6050_GYRO_CONFIG);
  Wire.write(0);
  Wire.endTransmission(true);

  // Set accelerometer range to +-16g
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(MPU6050_ACCEL_CONFIG);
  Wire.write(3<<3);
  Wire.endTransmission(true);

  return true;
}


void FeatherIMU::calibrateGyros() {
  gx_off = 0;
  gy_off = 0;
  gz_off = 0;

  for(int i = 0; i < 1000; i++) {
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPU6050_GYX);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDRESS, 6, true);

    gx_off += Wire.read() << 8 | Wire.read();
    gy_off += Wire.read() << 8 | Wire.read();
    gz_off += Wire.read() << 8 | Wire.read();
  }

  gx_off /= 1000;
  gy_off /= 1000;
  gz_off /= 1000;
}


void FeatherIMU::setInitialOrientation() {
  updateRawData();

  // Set angles using accelerometer
  roll = -180.0f * atan(az / sqrt(ay * ay + az * az)) / M_PI;
  pitch = 180.0f * atan(ay / sqrt(ax * ax + az * az)) / M_PI;
  heading = -180.0f * atan(az / sqrt(ax * ax + az * az)) / M_PI;

  // Copy to gyro angles
  gRoll = roll;
  gPitch = pitch;
  gYaw = heading;
}


void FeatherIMU::updateRawData() {
  Wire.beginTransmission(MPU6050_ADDRESS);
  // ACCEL_XOUT_H register
  // Tells the requestFrom to start reading from ax register 1 (16bit = 2 registers per reading)
  Wire.write(MPU6050_ACX);
  Wire.endTransmission(false);

  // Request data from 14 registers, starting from AccX as set above
  // True releases device from the bus
  Wire.requestFrom(MPU6050_ADDRESS, 14, true);
  ax = Wire.read() << 8 | Wire.read();  // ACCEL_XOUT_H(0x3B) && ACCEL_XOUT_L(0x3C)
  ay = Wire.read() << 8 | Wire.read();  // ACCEL_YOUT_H(0x3D) && ACCEL_YOUT_L(0x3E)
  az = Wire.read() << 8 | Wire.read();  // ACCEL_ZOUT_H(0x3F) && ACCEL_ZOUT_L(0x40)

  temperature = Wire.read() << 8 | Wire.read();  // TEMP_OUT_H(0x41) && TEMP_OUT_L(0x42)
  // Farenheight to Celsius
  temperature = (temperature / 340) + 36.53;

  gx = Wire.read() << 8 | Wire.read();  // GYRO_XOUT_H(0x43) && GYRO_XOUT_L(0x44)
  gy = Wire.read() << 8 | Wire.read();  // GYRO_YOUT_H(0x45) && GYRO_YOUT_L(0x46)
  gz = Wire.read() << 8 | Wire.read();  // GYRO_ZOUT_H(0x47) && GYRO_ZOUT_L(0x48)

  // Convert to 3D euler angles (accel) and degrees/s (gyro)

  // Accelerometers
  float scale = 9.81f * (16.0f / 32768);
  ax *= scale;
  ay *= scale;
  az *= scale;
  //data.ax = 57.295 * atan((float)data.ay / sqrt(pow((float)data.az, 2) + pow(data.ax, 2)));
  //data.ay = 57.295 * atan((float)-data.ax / sqrt(pow((float)data.az, 2) + pow(data.ay, 2)));

  // Gyros
  gx -= gx_off;
  gx /= GYRO_SENSITIVITY;
  gy -= gy_off;
  gy /= GYRO_SENSITIVITY;
  gz -= gz_off;
  gz /= GYRO_SENSITIVITY;
}


void FeatherIMU::updateAngles() {
  static uint32_t prevMicros = 0;
  if (prevMicros == 0) prevMicros = micros();

  float dt = (micros() - prevMicros) * 1e-6f;
  prevMicros = micros();

  gRoll += gy * dt;
  gPitch += gx * dt;
  gYaw += gx * dt;

  aRoll = -180.0f * atan(ax / sqrt(ay * ay + az * az)) / M_PI;
  aPitch = 180.0f * atan(ay / sqrt(ax * ax + az * az)) / M_PI;
  aYaw = -180.0f * atan(az / sqrt(ax * ax + az * az)) / M_PI;

  roll = (0.98f * gRoll) + (0.02f * aRoll);
  pitch= (0.98f * gPitch) + (0.02f * aPitch);
  heading = (0.98f * gYaw) + (0.02f * aYaw);

  // Compensate for gyro drift
  gRoll = roll;
  gPitch = pitch;
  gYaw = heading;
}


void FeatherIMU::update() {
  updateRawData();
  updateAngles();
}

void FeatherIMU::getRPH(float *r, float *p, float *h)
{
  *r = roll;
  *p = pitch;
  *h = heading;
}


void FeatherIMU::getAccRPY(float *r, float *p, float *y)
{
  *r = aRoll;
  *p = aPitch;
  *y = aYaw;
}


void FeatherIMU::getGyroRPY(float *r, float *p, float *y)
{
  *r = gRoll;
  *p = gPitch;
  *y = gYaw;
}
