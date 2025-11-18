#ifndef DRONEARMS_H
#define DRONEARMS_H

struct Arm {
  int length;
  int width;
  Servo motor;
}Arm;

#define X 8,5
#define Y 10,5
#define NUM_MOTOR 4

#endif