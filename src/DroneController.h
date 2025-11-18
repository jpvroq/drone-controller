#ifndef DRIVERCONTROLLER_H
#define DRIVERCONTROLLER_H

#include <ESP32Servo.h>

#include "PinUtils.h"

//#include "DroneArms.h"

class Arm {
  public:
  Servo motor;
  int pwm = 1000;
  float xPos;
  float yPos;
};

class Driver {
  private:
  Servo motors[NUM_MOTOR];
  int pwms[NUM_MOTOR];
  float P = 2, I = 0, D = 0;
  unsigned long timer[3] = {0, 0, 0};
  float prevIpart[3] = {0, 0, 0};
  float prevError[3] = {0, 0, 0};
  int correct(int pwm);

  public:
  void init();
  void secDetach();
  void calibrate();
  void set(int code, int pwm);
  void apply();
  void applyZero();
  float anglePID(float error, int axis);
  float ratePID(float error);
  void motorPWMs(float throttle, float roll, float pitch, float yaw);
};

#endif