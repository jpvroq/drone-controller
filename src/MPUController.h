#ifndef GYROSCOPECONROLLER_H
#define GYROSCOPECONTROLLER_H

#include "MPU6050.h"

class Gyroscope{
  private:
  float correctionMatrix[3][3] = {{0.248924, -0.002740, -0.000389},
                                  {-0.002740, 0.250667, -0.000701},
                                  {-0.000389, -0.000701, 0.251597}};
  float biasCorrection[3] = {-0.007103, -0.004587, 0.250670};
  unsigned long timer = 0;
  float anglePitch, angleRoll, pitchOffset = 0, rollOffset = 0, yawOffset = 0;
  float gyroPrevPitchAngle = 0, gyroPrevRollAngle = 0, gyroPrevYawAngle = 0;
  // Kalman output angle prediction/uncertainty (assumed to be of 2º)
  float kalmanOutput[2][2] = {{0, 2*2}, {0, 2*2}};
  void accelOutput(int16_t data[]);
  void gyroOutput(int16_t dara[]);
  void prepareModuleForOutput();
  

  public:
  void begin();
  void gyroAngle(float angle[]);
  void accelRates(float accRate[]);
  void calibratedAccelRates(float processedRates[]);
  void accelAngle(float angle[]);
  void calibrateGyro();
  void gyroRates(float rate[]);

  void kalmanAngle(float computedAngles[]);
};

#endif