#include <Wire.h>
#include "FeatherIMU.hpp"


#define MPU6050_ADDRESS 0x68
#define MPU6050_WHO_AM_I 0x75
#define MPU6050_ACX 0x3B
#define MPU6050_ACY 0x3D
#define MPU6050_ACZ 0x3F
#define MPU6050_TEMP 0x41
#define MPU6050_GYX 0X43
#define MPU6050_GYY 0X45
#define MPU6050_GYZ 0X47
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_PWR_MGMT_2 0x6C

#define GYRO_SENSITIVITY 131


FeatherIMU IMU;


bool FeatherIMU::init() 
{
  Wire.begin();
  Wire.beginTransmission(MPU6050_ADDRESS);
  // PWR_MGMT_1 register
  // Set to 0 to wake MPU6050
  Wire.write(MPU6050_PWR_MGMT_1);
  Wire.write(0);
  Wire.endTransmission(true);
  
  Wire.beginTransmission(MPU6050_ADDRESS);
  // WHO_AM_I register
  // Returns address of device
  Wire.write(MPU6050_WHO_AM_I);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDRESS, 1, true);
  int8_t whoAmI = Wire.read();

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
  Temp = 0;
  GyX_offset = 0;
  GyY_offset = 0;
  GyZ_offset = 0;
  gravity_angle = 0;
  calibrateGyros();
  
  return (whoAmI == MPU6050_ADDRESS) ? true : false;  
}


int16_t FeatherIMU::calibrateGravity() 
{
  calibrateGyros();
  // Read acceleration to find direction of gravity
  // Store angle for later
}


void FeatherIMU::updateData() 
{
  updateRaw();
  // Use gyros to shift acceleration xyz frame to be relative to gravity_angle
  // Extract vertical component
  // Extract roll pitch heading
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
  FeatherIMU::AcX = Wire.read() << 8 | Wire.read();  // ACCEL_XOUT_H(0x3B) && ACCEL_XOUT_L(0x3C)
  FeatherIMU::AcY = Wire.read() << 8 | Wire.read();  // ACCEL_YOUT_H(0x3D) && ACCEL_YOUT_L(0x3E)
  FeatherIMU::AcZ = Wire.read() << 8 | Wire.read();  // ACCEL_ZOUT_H(0x3F) && ACCEL_ZOUT_L(0x40)
  
  FeatherIMU::Temp = Wire.read() << 8 | Wire.read();  // TEMP_OUT_H(0x41) && TEMP_OUT_L(0x42)
  // Farenheight to Celsius
  FeatherIMU::Temp = (FeatherIMU::Temp / 340) + 36.53;
  
  FeatherIMU::GyX = Wire.read() << 8 | Wire.read();  // GYRO_XOUT_H(0x43) && GYRO_XOUT_L(0x44)
  FeatherIMU::GyY = Wire.read() << 8 | Wire.read();  // GYRO_YOUT_H(0x45) && GYRO_YOUT_L(0x46)
  FeatherIMU::GyZ = Wire.read() << 8 | Wire.read();  // GYRO_ZOUT_H(0x47) && GYRO_ZOUT_L(0x48)
    
  // Convert to 3D euler angles (accel) and degrees/s (gyro)
  
  // Accelerometers
  //FeatherIMU::AcX = 57.295 * atan((float)FeatherIMU::AcY / sqrt(pow((float)FeatherIMU::AcZ, 2) + pow(FeatherIMU::AcX, 2)));
  //FeatherIMU::AcY = 57.295 * atan((float)-FeatherIMU::AcX / sqrt(pow((float)FeatherIMU::AcZ, 2) + pow(FeatherIMU::AcY, 2)));
  
  // Gyros
  FeatherIMU::GyX -= GyX_offset;
  FeatherIMU::GyX /= GYRO_SENSITIVITY;
  FeatherIMU::GyY -= GyY_offset;
  FeatherIMU::GyY /= GYRO_SENSITIVITY;
  FeatherIMU::GyZ -= GyZ_offset;
  FeatherIMU::GyZ /= GYRO_SENSITIVITY;
}

