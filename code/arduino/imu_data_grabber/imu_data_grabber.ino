#include <Wire.h>


#define BAUD_RATE  115200


#define MPU6050_ADDRESS 0x68

#define MPU6050_WHO_AM_I 0x75

#define MPU6050_SMPRT_DIV     0x19
#define MPU6050_CONFIG        0x1A
#define MPU6050_GYRO_CONFIG   0x1B
#define MPU6050_ACCEL_CONFIG  0x1C

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


struct raw_data {
  float AcX, AcY, AcZ;
  float GyX, GyY, GyZ;
  float Temp;
};

struct angular_data {
  float gRoll, gPitch, gYaw;
  float aRoll, aPitch, aYaw;
  float roll, pitch, heading;
} angs;

int GyX_offset;
int GyY_offset;
int GyZ_offset;

String rxString;
bool rxMsgComplete = false;

bool initMPU6050();
void calibrateGyros();
raw_data getSensorReadings();
void rawDataToAngles(raw_data& data);
void printData(raw_data data);


void setup() {
  // Initialise serial comms
  Serial.begin(BAUD_RATE);
  rxString.reserve(256);
  Serial.println("\rSerial initialised");

  // Initialise I2C
  Wire.begin();
  Serial.println("I2C initialised");
  
  Serial.print("Initialising MPU6050... ");
  if(!initMPU6050()) {
    Serial.println("Failed!"); 
  } else {
    Serial.println("Initialised");
    delay(500);
    Serial.print("Calibrating gyros... ");
    calibrateGyros();
    Serial.println("Calibrated");
  }
  
  
  
  /*
  // Search for MPU6050
  Serial.print("Detecting MPU6050... ");
  Wire.beginTransmission(0x68);
  Serial.println(!Wire.endTransmission() ? "Found" : "Not found!");
  // Search for HMC5883L
  Serial.print("Detecting HMC5883L... ");
  Wire.beginTransmission(0x77);
  Serial.println(!Wire.endTransmission() ? "Found" : "Not found!");
  */
  
  // Initialise GPIO
  // LED
  pinMode(13, OUTPUT);
}

void loop() {
  if(rxMsgComplete) {
    Serial.println(rxString);
  }
  raw_data data = getSensorReadings();
  rawDataToAngles(data);
  //printData(data);
  
//  static int16_t maxAcX = 0;
//  maxAcX = data.AcX > maxAcX ? data.AcX : maxAcX;
//  Serial.print(data.AcX);
//  Serial.print(" (");
//  Serial.print(maxAcX);
//  Serial.print(") ");
//
//  static int16_t maxAcY = 0;
//  maxAcY = data.AcY > maxAcY ? data.AcY : maxAcY;
//  Serial.print(data.AcY);
//  Serial.print(" (");
//  Serial.print(maxAcY);
//  Serial.println(") ");

//  Serial.print("r: ");
//  Serial.print(angs.roll);
//  Serial.print("   p: ");
//  Serial.print(angs.pitch);
//  //Serial.print("   heading: ");
//  //Serial.println(angs.heading);
//  Serial.print("   gr: ");
//  Serial.print(angs.gRoll);
//  Serial.print("   gp: ");
//  Serial.print(angs.gPitch);
//  Serial.print("   ar: ");
//  Serial.print(angs.aRoll);
//  Serial.print("   ap: ");
//  Serial.println(angs.aPitch);
  Serial.print(angs.roll);
  Serial.print(" ");
  Serial.println(angs.pitch);
  


  // Toggle LED to show activity
  static bool ledState = false;
  digitalWrite(13, ledState ? LOW : HIGH);
  ledState = !ledState;
  //delay(100);
}


void serialEvent() {
  while(Serial.available()) {
    char c = (char)Serial.read();
    
    rxString += c;
    
    if(c == '\n') {
      rxMsgComplete = true;
    }
    
  } 
}


bool initMPU6050()
{
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


void calibrateGyros()
{
  GyX_offset = 0;
  GyY_offset = 0;
  GyZ_offset = 0;

  for(int i = 0; i < 1000; i++) {
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
  
  Serial.println("Gyros calibrated.");
  Serial.print("Gyro X offset = "); Serial.println(GyX_offset);
  Serial.print("Gyro Y offset = "); Serial.println(GyY_offset);
  Serial.print("Gyro Z offset = "); Serial.println(GyZ_offset);
  Serial.println();


  raw_data data = getSensorReadings();
  angs.roll = -180.0f * atan(data.AcX / sqrt(data.AcY * data.AcY + data.AcZ * data.AcZ)) / M_PI;
  angs.pitch = 180.0f * atan(data.AcY / sqrt(data.AcX * data.AcX + data.AcZ * data.AcZ)) / M_PI;
  angs.heading = -180.0f * atan(data.AcZ / sqrt(data.AcX * data.AcX + data.AcZ * data.AcZ)) / M_PI;
  angs.gRoll = angs.roll;
  angs.gPitch = angs.pitch;
  angs.gYaw = angs.heading;
}


raw_data getSensorReadings()
{
  raw_data data;
  
  Wire.beginTransmission(MPU6050_ADDRESS);
  // ACCEL_XOUT_H register
  // Tells the requestFrom to start reading from AcX register 1 (16bit = 2 registers per reading)
  Wire.write(MPU6050_ACX);
  Wire.endTransmission(false);
  
  // Request data from 14 registers, starting from AccX as set above
  // True releases device from the bus
  Wire.requestFrom(MPU6050_ADDRESS, 14, true);
  data.AcX = Wire.read() << 8 | Wire.read();  // ACCEL_XOUT_H(0x3B) && ACCEL_XOUT_L(0x3C)
  data.AcY = Wire.read() << 8 | Wire.read();  // ACCEL_YOUT_H(0x3D) && ACCEL_YOUT_L(0x3E)
  data.AcZ = Wire.read() << 8 | Wire.read();  // ACCEL_ZOUT_H(0x3F) && ACCEL_ZOUT_L(0x40)
  
  data.Temp = Wire.read() << 8 | Wire.read();  // TEMP_OUT_H(0x41) && TEMP_OUT_L(0x42)
  // Farenheight to Celsius
  data.Temp = (data.Temp / 340) + 36.53;
  
  data.GyX = Wire.read() << 8 | Wire.read();  // GYRO_XOUT_H(0x43) && GYRO_XOUT_L(0x44)
  data.GyY = Wire.read() << 8 | Wire.read();  // GYRO_YOUT_H(0x45) && GYRO_YOUT_L(0x46)
  data.GyZ = Wire.read() << 8 | Wire.read();  // GYRO_ZOUT_H(0x47) && GYRO_ZOUT_L(0x48)
    
  // Convert to 3D euler angles (accel) and degrees/s (gyro)
  
  // Accelerometers
  float scale = 9.81f * (16.0f / 32768);
  data.AcX *= scale;
  data.AcY *= scale;
  data.AcZ *= scale;
  //data.AcX = 57.295 * atan((float)data.AcY / sqrt(pow((float)data.AcZ, 2) + pow(data.AcX, 2)));
  //data.AcY = 57.295 * atan((float)-data.AcX / sqrt(pow((float)data.AcZ, 2) + pow(data.AcY, 2)));
  
  // Gyros
  data.GyX -= GyX_offset;
  data.GyX /= GYRO_SENSITIVITY;
  data.GyY -= GyY_offset;
  data.GyY /= GYRO_SENSITIVITY;
  data.GyZ -= GyZ_offset;
  data.GyZ /= GYRO_SENSITIVITY;
  
  
  return data;
}


void rawDataToAngles(raw_data& data)
{  
  static uint32_t prevMicros = 0;
  if (prevMicros == 0) prevMicros = micros();

  float dt = (micros() - prevMicros) * 1e-6f;
  prevMicros = micros();

  angs.gRoll += data.GyY * dt;
  angs.gPitch += data.GyX * dt;
  angs.gYaw += data.GyZ * dt;

  angs.aRoll = -180.0f * atan(data.AcX / sqrt(data.AcY * data.AcY + data.AcZ * data.AcZ)) / M_PI;
  angs.aPitch = 180.0f * atan(data.AcY / sqrt(data.AcX * data.AcX + data.AcZ * data.AcZ)) / M_PI;
  angs.aYaw = -180.0f * atan(data.AcZ / sqrt(data.AcX * data.AcX + data.AcZ * data.AcZ)) / M_PI;

  angs.roll = (0.98f * angs.gRoll) + (0.02f * angs.aRoll);
  angs.pitch= (0.98f * angs.gPitch) + (0.02f * angs.aPitch);
  angs.heading = (0.98f * angs.gYaw) + (0.02f * angs.aYaw);

  // Compensate for gyro drift
  angs.gRoll = angs.roll;
  angs.gPitch = angs.pitch;
  angs.gYaw = angs.heading;
}


void printData(raw_data data)
{
  Serial.print("AcX = "); Serial.print(data.AcX);
  Serial.print("| AcY = "); Serial.print(data.AcY);
  Serial.print("| AcZ = "); Serial.println(data.AcZ);
  
  Serial.print("GyX = "); Serial.print(data.GyX);
  Serial.print("| GyY = "); Serial.print(data.GyY);
  Serial.print("| GyZ = "); Serial.println(data.GyZ);
  
  Serial.print("Temp = "); Serial.print(data.Temp); Serial.println(" Celsius");
}
