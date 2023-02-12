/* Gyroscope / Accelerometer Test
 *  MPU-6050 IMU
*/

#include <Wire.h>  // Include Wire library for I2C


// Gyroscope variables
int gyroX, gyroY, gyroZ;
long gyroXCAL, gyroYCAL, gyroZCAL;
boolean setGyroAngles;

long accX, accY, accZ, accTotalVector;
float angleRollAcc, anglePitchAcc;
// float angleYaw;  // Not using yaw due to angle drift.

float anglePitch, angleRoll;
int anglePitchBuffer, angleRollBuffer;
float anglePitchOutput, angleRollOutput;

// Timer and loop variables
long loopTimer;
int temp;

// Display counter
int displayCount = 0;

// Number of samples
int samples = 1000;

void setup() {
  Wire.begin(); // Start I2C

  MPURegisterSetup(); // Setup MPU registers
  readRawData(); // Read the raw acc and gyro data

  // Average out the offset
  gyroXCAL /= samples;
  gyroYCAL /= samples;
  gyroZCAL /= samples;

  Serial.begin(115200);

  loopTimer = micros();
}

void loop() {
  readMPUData();

  gyroX -= gyroXCAL;  // Offset the values
  gyroY -= gyroYCAL;
  gyroZ -= gyroZCAL;

  anglePitch += gyroX * 0.0000611;
  angleRoll += gyroY * 0.0000611;
  // angleYaw += gyroZ * dt; or 0.0000611?

  // 0.000001066 = 0.0000611 * (PI / 180deg)
  // If IMU has yawed, transfer the roll angle to the pitch angle
  anglePitch += angleRoll * sin(gyroZ * 0.000001066);
  // If IMU has yawed, transfer the pitch angle to the roll angle
  angleRoll += anglePitch * sin(gyroZ * 0.000001066);

  // 57.296 = 1 / (PI / 180);
  accTotalVector = sqrt(pow(accX, 2) + pow(accY, 2) + pow(accZ, 2));
  anglePitchAcc = asin((float)accY / accTotalVector) * 57.296;
  angleRollAcc = asin((float)accX / accTotalVector) * (-57.296);

  anglePitchAcc -= 0.0; // Accelerometer calibration value pitch
  angleRollAcc -= 0.0; // Accelerometer calibration value for roll

  if (setGyroAngles) {
    anglePitch = anglePitch * 0.9996 + anglePitchAcc * 0.0004;
    angleRoll = angleRoll * 0.9996 + angleRollAcc * 0.0004;
  }
  else {
    anglePitch = anglePitchAcc;
    angleRoll = angleRollAcc;
    setGyroAngles = true;
  }

  // 90% of the output value and add 10% of the raw value
  anglePitchOutput = anglePitchOutput * 0.9 + anglePitch * 0.1;
  angleRollOutput = angleRollOutput * 0.9 + angleRoll * 0.1;

  Serial.print("Pitch Angle = ");
  Serial.print(anglePitchOutput);
  Serial.print("  |  Roll Angle = ");
  Serial.println(angleRollOutput);

  // Wait until timer reaches 4ms (250 Hz) before next loop
  while (micros() - loopTimer < 4000);
  loopTimer = micros();

}
// ---------------------------------------------------------
void MPURegisterSetup() {
  //Activate MPU-6050
  Wire.beginTransmission(0x68); // Start communication with MPU-6050
  Wire.write(0x6B); // Send the requested starting register  *B NOT 8*
  Wire.write(0x00); // Set the requsted starting register
  Wire.endTransmission(); // End transmission

  // Configure the accelerometer (+/- 8g)
  Wire.beginTransmission(0x68); // Start communication
  Wire.write(0x1C); // Send the requested starting register
  Wire.write(0x10); // Set the requested starting register
  Wire.endTransmission(); // End transmission

  // Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68); // Start communication
  Wire.write(0x1B); // Send the requested starting register
  Wire.write(0x08); // Set the requested starting register
  Wire.endTransmission(); // End transmission
}

// Read the raw acc and gyro data from the MPU-6050 1000 times
void readRawData() {
  for (int i = 0; i < samples; i ++) {
    readMPUData();

    gyroXCAL += gyroX; // Add the gyro x offset to the gyroXCAL variable
    gyroYCAL += gyroY; // Add the gyro y offset to the gyroYCAL variable
    gyroZCAL += gyroZ; // Add the gyro z offset to the gyroZCAL variable
    delay(3); // Delay 3us to have 250 Hz for-loop
  }
}

// Get data from the MPU-6050
void readMPUData() {
  Wire.beginTransmission(0x68); // Start communication with MPU-6050
  Wire.write(0x3B); // Send the requested starting register
  Wire.endTransmission(); 
  Wire.requestFrom(0x68, 14); // Request 14 bytes from the MPU-6050
  while (Wire.available() < 14); // Wait until all bytes received

  // Shift left 8 bits then bitwise OR
  // Turns two 8-bit values into 16-bit value
  accX = Wire.read() << 8 | Wire.read();
  accY = Wire.read() << 8 | Wire.read();
  accZ = Wire.read() << 8 | Wire.read();
  temp = Wire.read() << 8 | Wire.read();
  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();
}
