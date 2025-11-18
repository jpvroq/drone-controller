#include "MPUController.h"

#include <Arduino.h>
#include <Wire.h>

void Gyroscope::begin() {
  // Set I2C and MPU clock rate
  Wire.setClock(400000);
  Wire.begin();
  // Wait for initialization of module
  delay(500);
  // Set 0x6b register to all 0 (deactivate reset, sleep, cycle, temp_dis and set clock to internal 8MHz)
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
}

void Gyroscope::prepareModuleForOutput() {
  // Default address for MPU6050 is 0x68
  Wire.beginTransmission(0x68);
  // Register for module configuration, activate low-pass filter for high frequency modulation
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
}

void Gyroscope::accelOutput(int16_t data[]) {

  prepareModuleForOutput();
  // Register for accelerometer configuration, set output units to +- 8 g
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  // Read raw data from register 0x3B (start of accelerometer data) to register 0x40 (end of accelerometer data)
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  // Request 6 registers (2 for each axis with 8 bits each)
  Wire.requestFrom(0x68, 6);
  for (int i = 0; i < 3; i++) {
    data[i] = Wire.read() << 8 | Wire.read(); 
  }
}

void Gyroscope::gyroOutput(int16_t data[]) {

  prepareModuleForOutput();
  // Gyroscope configuration
  Wire.beginTransmission(0x68);
  // Gyroscope configuration register is 0x1B
  Wire.write(0x1B);
  // Configure output in +-500 LSB/º/s
  Wire.write(0x08);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  // Read gyroscope raw data from register 43 to 48
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  for (int i = 0; i < 3; i++) {
    data[i] = Wire.read() << 8 | Wire.read();
  }
}

void Gyroscope::accelRates(float accRate[]) {
  int16_t accData[3];
  // From the acceleration rate, pitch in radiants is atan(accx / (sqrt(accy*accy + accz*accz)))
  // roll is atan(accy / sqrt(accx*accx + accz*accz))
  
  // Get accelerometer output
  accelOutput(accData);

  // Get acceleration
  // Raw accelerometer data is in LSB(least significant bit)/g (4096 LSB/g = 8 g)
  // TODO: add calibration to acc
  accRate[0] = (float) accData[0] / 4096;
  accRate[1] = (float) accData[1] / 4096;
  accRate[2] = (float) accData[2] / 4096;
}

void Gyroscope::calibratedAccelRates(float processedRates[]) {
  float rawRates[3];

  accelRates(rawRates);
  // Apply bias correction
  for (int i = 0; i < 3; i++) {
    rawRates[i] -= biasCorrection[i];
  }
  // Multiply matrix
  for (int i = 0; i < 3; i++) {
    float assist = 0;
    for (int j = 0; j < 3; j++) {
      assist += correctionMatrix[i][j] * rawRates[j];
    }
    processedRates[i] = assist;
  }
}

void Gyroscope::accelAngle(float angle[3]) {
  float accRate[3];

  accelRates(accRate);
  // Get current angles from accelerometer angle rates
  // Arctangent function output is in radiants ( º = rad* 1 /(pi/180))
  angle[0] = atan(accRate[0] / sqrt(accRate[1]*accRate[1] + accRate[2]*accRate[2])) * 1 / (3.1415 / 180);
  angle[1] = atan(accRate[1] / sqrt(accRate[0]*accRate[0] + accRate[2]*accRate[2])) * 1 / (3.1415 / 180);
  // This method cannot be used for calculating yaw angle
}

void Gyroscope::calibrateGyro()  {
  // Get the avarage of a sample and apply to offset
  const int SAMPLES = 2000;
  int16_t gyroData[3];
  // Offsets roll pitch yaw
  float offset[3] = {0, 0, 0};
  for (int i = 0; i < SAMPLES; i++) {
    gyroOutput(gyroData);
    offset[0] += (float)gyroData[0] / 65.5;
    offset[1] += (float)gyroData[1] / 65.5;
    offset[2] += (float)gyroData[2] / 65.5;
    delay(1);
  }
  offset[0] /= SAMPLES;
  offset[1] /= SAMPLES;
  offset[2] /= SAMPLES;
  rollOffset = offset[0];
  pitchOffset = offset[1];
  yawOffset = offset[2];
}

void Gyroscope::gyroRates(float rate[]) {
  int16_t gyroData[3];

  gyroOutput(gyroData);
  rate[1] = ((float)gyroData[0] / 65.5) - rollOffset;
  rate[0] = ((float)gyroData[1] / 65.5) - pitchOffset;
  rate[2] = ((float)gyroData[2] / 65.5) - yawOffset;
}

void Gyroscope::gyroAngle(float angle[]) {
  float rate[3];

  gyroRates(rate);
  int time = millis();
  gyroPrevPitchAngle = gyroPrevPitchAngle + rate[0] * ((time - timer) / 1000);
  gyroPrevRollAngle = gyroPrevRollAngle + rate[1] * ((time - timer) / 1000);
  gyroPrevYawAngle = gyroPrevYawAngle + rate[2] * ((time - timer) / 1000);
  timer = time;
  angle[0] = gyroPrevPitchAngle;
  angle[1] = gyroPrevRollAngle;
  angle[2] = gyroPrevYawAngle;
}

void Gyroscope::kalmanAngle(float computedAngles[]) {
  float rotationRate[3], accelAng[3];
  float newAngle, newUncertainty, gain;
  
  gyroRates(rotationRate);
  accelAngle(accelAng);
  unsigned long time = millis();
  int dt = (int)(time - timer);
  for (int i = 0; i < 2; i++) {
    // Prediction phase Anlge = prevAngle + dt * rate
    newAngle = kalmanOutput[i][0] + dt*rotationRate[i];
    // Uncertanty phase uncertanty = prevUncertainty * dt*dt*4*4
    newUncertainty = kalmanOutput[i][1] + dt * dt * 4 * 4;
    // Gain phase based on previous predictions
    gain = newUncertainty * (1 / (newUncertainty + 3 * 3));
    // Update phase
    newAngle = newAngle + gain * (accelAng[i] - newAngle);
    newUncertainty = (1 - gain) * newUncertainty;
    kalmanOutput[i][0] = newAngle;
    computedAngles[i] = newAngle;
    kalmanOutput[i][1] = newUncertainty;
  }
  timer = time;
}