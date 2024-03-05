#include <Wire.h>

#define MPU9250_ADDR 0x68 // MPU9250 I2C address

// MPU9250 register addresses
#define ACCEL_XOUT_H 0x3B
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F
#define GYRO_XOUT_H 0x43
#define GYRO_YOUT_H 0x45
#define GYRO_ZOUT_H 0x47

float pitchAngle = 0;
float yawAngle = 0;
const float alpha = 0.98; // Complementary filter coefficient

float accelX_mps2 = 0, accelY_mps2 = 0, accelZ_mps2 = 0;
float gyroX_dps = 0, gyroY_dps = 0, gyroZ_dps = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  initializeMPU9250();
}

void loop() {
  readMPU9250();
  calculateAngles();

  Serial.print("Pitch: ");
  Serial.print(pitchAngle);
 Serial.print(" degrees, Yaw: ");
 Serial.print(yawAngle);
  Serial.println(" degrees");

  delay(10);
}

void initializeMPU9250() {
  // Initialize MPU9250 sensor
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // Wake up MPU9250
  Wire.endTransmission(true);
}

void readMPU9250() {
  // Read accelerometer data
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU9250_ADDR, 6, true);
  int16_t accelX = Wire.read() << 8 | Wire.read();
  int16_t accelY = Wire.read() << 8 | Wire.read();
  int16_t accelZ = Wire.read() << 8 | Wire.read();

  // Read gyroscope data
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(GYRO_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU9250_ADDR, 6, true);
  int16_t gyroX = Wire.read() << 8 | Wire.read();
  int16_t gyroY = Wire.read() << 8 | Wire.read();
  int16_t gyroZ = Wire.read() << 8 | Wire.read();

  // Convert raw data to physical units and store in global variables
  float accelScale = 9.81 / 16384.0; // LSB/g
  float gyroScale = 250.0 / 32768.0; // LSB/deg/s
  accelX_mps2 = accelX * accelScale;
  accelY_mps2 = accelY * accelScale;
  accelZ_mps2 = accelZ * accelScale;
  gyroX_dps = gyroX * gyroScale;
  gyroY_dps = gyroY * gyroScale;
  gyroZ_dps = gyroZ * gyroScale;
}

void calculateAngles() {
  // Calculate pitch angle
  float pitchAcc = atan2(accelY_mps2, sqrt(pow(accelX_mps2, 2) + pow(accelZ_mps2, 2))) * 180 / M_PI;
  pitchAngle = alpha * (pitchAngle + gyroX_dps * 0.01) + (1 - alpha) * pitchAcc;

  // Calculate yaw angle
  float yawAcc = atan2(accelX_mps2, sqrt(pow(accelY_mps2, 2) + pow(accelZ_mps2, 2))) * 180 / M_PI;
  yawAngle = alpha * (yawAngle + gyroZ_dps * 0.01) + (1 - alpha) * yawAcc;
}

