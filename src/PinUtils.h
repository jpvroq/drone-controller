#ifndef PINUTILS_H
#define PINUTILS_H

// PWM ESC PINS
#define BACK_RIGHT_MOTOR D7
#define BACK_LEFT_MOTOR A2
#define FRONT_RIGHT_MOTOR A0
#define FRONT_LEFT_MOTOR D9

// RADIO PIN IBUS PROTOCOL
#define IBUS_RX_PIN 14

// IBUS CHANNELS
#define YAW 0
#define PITCH 1
#define ROLL 3
#define THROTTLE 2
#define SWA 4       //Arm
#define SWB 5       // Calibrate
#define SWC 6
#define SWD 7
#define DIAL_A 8 
#define DIAL_B 9

// Arm data
#define X 8,5
#define Y 10,5
#define NUM_MOTOR 4

// PWM CONSTANTS
#define MIN_US 1000
#define MAX_US 2000

//PID equation constants
#define PID_P 1
#define PID_I 1
#define PID_D 1

//debugging
#define DEBUG false

#endif