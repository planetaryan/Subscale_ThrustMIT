#include <Wire.h>
#include <Servo.h>

// Declare Servo objects for the left and right propellers
Servo right_prop;
Servo left_prop;

// Variables to store raw accelerometer and gyroscope readings
int16_t Acc_rawX, Acc_rawY, Acc_rawZ;
int16_t Gyr_rawX, Gyr_rawY;

// Arrays to store calculated angles
float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];
float Gyro[2];

// Variables for timing
float elapsedTime, time_, timePrev;

// Conversion factor for radians to degrees
float rad_to_deg = 180/3.141592654;

// Variables to store servo corrections and PWM values
float correction_left;
float correction_right;
float pwmLeft, pwmRight;

void setup() {
  // Initialize I2C communication and MPU9250 sensor
  Wire.begin();
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  // Initialize serial communication
  Serial.begin(115200);

  // Attach servos to corresponding pins
  right_prop.attach(9);
  left_prop.attach(11);

  // Initialize servos to initial position
  left_prop.writeMicroseconds(1000); 
  right_prop.writeMicroseconds(1000);

  // Delay for sensor stabilization
  delay(200);
}

void loop() {
  // Record previous time and current time for time calculation
  timePrev = time_;
  time_ = millis();
  elapsedTime = (time_ - timePrev) / 1000;

  // Read accelerometer data
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 6, true);

  // Read accelerometer data for X, Y, Z axes
  Acc_rawX = Wire.read() << 8 | Wire.read();
  Acc_rawY = Wire.read() << 8 | Wire.read();
  Acc_rawZ = Wire.read() << 8 | Wire.read();

  // Calculate pitch and roll angles using accelerometer data
  Acceleration_angle[0] = atan((Acc_rawY/16384.0) / sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2))) * rad_to_deg;
  Acceleration_angle[1] = atan(-1 * (Acc_rawX/16384.0) / sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2))) * rad_to_deg;

  // Read gyroscope data
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 4, true);

  // Read gyroscope data for X, Y axes
  Gyr_rawX = Wire.read() << 8 | Wire.read();
  Gyr_rawY = Wire.read() << 8 | Wire.read();

  // Convert gyroscope readings to degrees per second
  Gyro_angle[0] = Gyr_rawX / 131.0; 
  Gyro_angle[1] = Gyr_rawY / 131.0;

  // Combine accelerometer and gyroscope angles using complementary filter
  Gyro[0] = Acceleration_angle[0] + Gyro_angle[0] * elapsedTime;   
  Gyro[1] = Acceleration_angle[1] + Gyro_angle[1] * elapsedTime;
  
  // Calculate total angles by applying complementary filter
  Total_angle[0] = 0.98 * Gyro[0] + 0.02 * Acceleration_angle[0];
  Total_angle[1] = 0.98 * Gyro[1] + 0.02 * Acceleration_angle[1];
  
  // Print total angle corrections to serial monitor
  Serial.print("cXAngle = ");
  Serial.print(correction_left);
  Serial.print("\n");
  Serial.print("cYAngle = ");
  Serial.print(correction_right);
  Serial.print("\n");
  
  // Calculate servo corrections for pitch and roll
  correction_left = 0 - Total_angle[0];
  correction_right = 0 - Total_angle[1];
  
  // Map corrections to PWM values for servos
  pwmLeft = map(correction_left, -45, 45, 1000, 2000);
  pwmRight = map(correction_right, -45, 45, 1000, 2000);

  // Constrain PWM values to valid range
  pwmLeft = constrain(pwmLeft, 1000, 2000);
  pwmRight = constrain(pwmRight, 1000, 2000);

  // Send PWM signals to servos
  left_prop.writeMicroseconds(pwmLeft);
  right_prop.writeMicroseconds(pwmRight);

  // Delay for stability
  delay(300); 
}
