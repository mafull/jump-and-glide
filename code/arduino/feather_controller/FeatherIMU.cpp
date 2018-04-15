#include <Wire.h>
#include <math.h>
#include "FeatherIMU.hpp"


#define MPU6050_ADDRESS 0x68

#define MPU6050_SMPRT_DIV     0x19
#define MPU6050_CONFIG        0x1A
#define MPU6050_GYRO_CONFIG   0x1B
#define MPU6050_ACCEL_CONFIG  0x1C
#define MPU6050_ACX           0x3B
#define MPU6050_ACY           0x3D
#define MPU6050_ACZ           0x3F
#define MPU6050_TEMP          0x41
#define MPU6050_GYX           0X43
#define MPU6050_GYY           0X45
#define MPU6050_GYZ           0X47
#define MPU6050_PWR_MGMT_1    0x6B
#define MPU6050_WHO_AM_I      0x75


FeatherIMU IMU;


bool FeatherIMU::init() 
{
  // Initialise variables
  roll = 0;
  pitch = 0;
  heading = 0;
  vertical_acc = 0;
  AcX = 0;
  AcY = 0;
  AcZ = 0;
  GyX = 0;
  GyY = 0;
  GyZ = 0;
  GyX_offset = 0;
  GyY_offset = 0;
  GyZ_offset = 0;
  gravity_angleX = 0;
  gravity_angleY = 0;
  gravity_angleZ = 0;
  calibrateGyros();

  // Initialise I2C
  Wire.begin();

  // Set up IMU
  // Wake up
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(MPU6050_PWR_MGMT_1);
  Wire.write(0);
  Wire.endTransmission(true);
  
  // Check WHO_AM_I
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(MPU6050_WHO_AM_I);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDRESS, 1, true);
  if(Wire.read() != MPU6050_ADDRESS) return false;
    
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


void FeatherIMU::calibrateGravity() 
{
  calibrateGyros();
  updateRaw();
  gravity_angleX = atan2(AcZ,AcY);
  gravity_angleY = atan2(AcX,AcZ);
  gravity_angleZ = atan2(AcY,AcX);
}


void FeatherIMU::updateData()
{
  updateRaw();
  // Extract magnitude of acceleration in vertical direction, and remove gravity.
  vertical_acc = sqrt(AcX*AcX + AcY*AcY + AcZ*AcZ)*sin(GyX + gravity_angleX) - GRAVITY;
  roll = GyX;
  pitch = GyY;
  heading = GyZ;
}


void FeatherIMU::calibrateGyros()
{
  GyX_offset = 0;
  GyY_offset = 0;
  GyZ_offset = 0;

  for(int i = 0; i < 1000; i++) 
  {
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPU6050_GYX);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDRESS, 6, true);
    
    GyX_offset += Wire.read() << 8 | Wire.read();
    GyY_offset += Wire.read() << 8 | Wire.read();
    GyZ_offset += Wire.read() << 8 | Wire.read();
  }
  
  GyX_offset /= 1000;
  GyY_offset /= 1000;
  GyZ_offset /= 1000;
}


void FeatherIMU::updateRaw() 
{
  Wire.beginTransmission(MPU6050_ADDRESS);
  // ACCEL_XOUT_H register
  // Tells the requestFrom to start reading from AcX register 1 (16bit = 2 registers per reading)
  Wire.write(MPU6050_ACX);
  Wire.endTransmission(false);
  
  // Request data from 14 registers, starting from AccX as set above
  // True releases device from the bus
  Wire.requestFrom(MPU6050_ADDRESS, 14, true);
  AcX = Wire.read() << 8 | Wire.read();  // ACCEL_XOUT_H(0x3B) && ACCEL_XOUT_L(0x3C)
  AcY = Wire.read() << 8 | Wire.read();  // ACCEL_YOUT_H(0x3D) && ACCEL_YOUT_L(0x3E)
  AcZ = Wire.read() << 8 | Wire.read();  // ACCEL_ZOUT_H(0x3F) && ACCEL_ZOUT_L(0x40)
  
  GyX = Wire.read() << 8 | Wire.read();  // GYRO_XOUT_H(0x43) && GYRO_XOUT_L(0x44)
  GyY = Wire.read() << 8 | Wire.read();  // GYRO_YOUT_H(0x45) && GYRO_YOUT_L(0x46)
  GyZ = Wire.read() << 8 | Wire.read();  // GYRO_ZOUT_H(0x47) && GYRO_ZOUT_L(0x48)
    
  // Scale data
  // Accelerometers, +-16g to m/s2
  float scale = GRAVITY*16/32768;
  AcX = scale*AcX;
  AcY = scale*AcY;
  AcZ = scale*AcZ;

  // Gyros, +-250deg/s to radians/sec
  scale = (M_PI/180)*250/32768;
  GyX = scale*(GyX-GyX_offset);
  GyY = scale*(GyY-GyY_offset);
  GyZ = scale*(GyZ-GyZ_offset);
}

