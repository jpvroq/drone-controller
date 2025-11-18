#include "DroneController.h"

#include "PinUtils.h"

void Driver::init() {
  // The drone used to test the algorithm has 4 arms, each equidistant to the geometrical center.
  //arm = new Arm[NUM_MOTOR];
  secDetach();
  /*
  for (int i = 0; i < NUM_MOTOR; i++) {
    arm[i].xPos = (i < 2) ? X : -1*X;
    arm[i].yPos = (i % 2 == 0 ) ? Y : -1*Y;
  }
  */
}

void Driver::secDetach() {
  for (int i = 0; i < NUM_MOTOR; i++) {
    //arm[i].motor.detach();
    motors[i].detach();
  }
}

void Driver::calibrate() {
  int pins[NUM_MOTOR] = {FRONT_LEFT_MOTOR, BACK_LEFT_MOTOR, BACK_RIGHT_MOTOR, FRONT_RIGHT_MOTOR};
  secDetach();
  for (int i = 0; i < NUM_MOTOR; i++){
    /*
    arm[i].motor.attach(pins[i], 1000, 2000);
    arm[i].motor.writeMicroseconds(2000);
    */
    motors[i].attach(pins[i], 1000, 2000);
    motors[i].writeMicroseconds(2000);
  }
  delay(2000);
  for (int i = 0; i < NUM_MOTOR; i++){
    //arm[i].motor.writeMicroseconds(1000);
    motors[i].writeMicroseconds(1000);
  }
  delay(1000);
}

void Driver::set(int code, int pwm) {
  if (pwm < 1000) pwm = 1000;
  else if (pwm >= 2000) pwm = 1999;
  //arms[code].motor.writeMicroseconds(pwm);
  motors[code].writeMicroseconds(pwm);
}

void Driver::apply() {
  for (int i = 0; i < NUM_MOTOR; i++) {
    //arm[i].motor.writeMicroseconds(pwm[i]);
    if (DEBUG) Serial.printf("Applyed motor %d\n", i);
    motors[i].writeMicroseconds(pwms[i]);
  }
}

void Driver::applyZero() {
  for (int i = 0; i < NUM_MOTOR; i++) {
    //arm[i].motor.writeMicroseconds(1000);
    motors[i].writeMicroseconds(1000);
  }
}

int Driver::correct(int pwm) {
  if (pwm < 1000) return 1000;
  if (pwm >= 2000) return 1999;
  return pwm;
}

// PID based of predicted angle (for pitch and roll)
float Driver::anglePID(float error, int axis) {
  //
  float Ppart = P * error;
  unsigned long time = millis();
  float Ipart = prevIpart[axis] + I * (error + prevError[axis]) * ((float)(time - timer[axis]) / 2);
  if (Ipart < -400) Ipart = -400;
  else if (Ipart > 400) Ipart = 400;
  float Dpart = D * (error - prevError[axis]) / (float)(time - timer[axis]);
  float PID = Ppart + Ipart + Dpart;
  if (PID < -400) PID = -400;
  else if (PID > 400) PID = 400;
  prevError[axis] = error;
  prevIpart[axis] = Ipart;
  timer[axis] = time;
  return PID;
}

// PID based of angle rate (for yaw)
float Driver::ratePID(float error) {
  unsigned long time = millis();
  float Ppart = P * error;
  float Ipart = prevIpart[2] + I * (error + prevError[2]) * ((float)(time - timer[2]) / 2);
  if (Ipart > 400) Ipart = 400;
  else if (Ipart < -400) Ipart = -400;
  float Dpart = D * (error-prevError[3]) / (float)(time - timer[2]);
  prevError[2] = error;
  prevIpart[2] = Ipart;
  timer[2] = time;
  return Ppart + Ipart + Dpart;
}

void Driver::motorPWMs(float throttle, float roll, float pitch, float yaw) {
  pwms[0] = correct((int) ((throttle - roll - pitch - yaw)));
  pwms[1] = correct((int) ((throttle - roll + pitch + yaw)));
  pwms[2] = correct((int) ((throttle + roll + pitch - yaw)));
  pwms[3] = correct((int) ((throttle + roll - pitch + yaw)));
}