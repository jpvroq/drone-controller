#include <Arduino.h>
#include <Wire.h>

#include "IBusBM.h"

#include "MPUController.h"
#include "DroneController.h"
#include "RadioController.h"

#include "PinUtils.h"

int availableMemory();
void execController();
void controllerInitialization();

Radio radio;
Gyroscope gyro;
Driver driver;

bool calibrated = false;

void setup() {
  if (DEBUG) Serial.begin();
  if (DEBUG) while(!Serial);
  // ESC initialization
  if (DEBUG) Serial.println("Driver initialization.");
  driver.init();
  // MPU module initialization
  if (DEBUG) Serial.println("Initializing gyroscope...");
  gyro.begin();
  // Radio Initialization
  if (DEBUG) Serial.println("Radio initialization");
  radio.begin(Serial1, 0, D4, -1);
  while(!Serial1);

  // Set LED modes for state visualization
  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_RED, HIGH);
  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_GREEN, HIGH);

  if (DEBUG) Serial.println("Setup completed.");
  int swa = radio.getChannel(SWA);
  int swb = radio.getChannel(SWB);
  if (DEBUG) Serial.printf("%d, %d\n", swa, swb);
  while (swa > 1500 && swb > 1500) {
    digitalWrite(LED_RED, LOW);
    delay(500);
    digitalWrite(LED_RED, HIGH);
    swa = radio.getChannel(SWA);
    swb = radio.getChannel(SWB);
  }
}

void loop() {
  // Get throttle and desired angles
  digitalWrite(LED_GREEN, LOW);
  
  int swa = radio.getChannel(SWA);
  int swb = radio.getChannel(SWB);

  if (swa > 1500) {
    if (swb > 1500 and !calibrated) {
      controllerInitialization();
    } else if(calibrated) {
      execController();
    }
  }
  digitalWrite(LED_GREEN, HIGH);
}

int availableMemory() {
  int size = 8192;
  byte *buf;
  while ((buf = (byte *) malloc(--size)) == NULL);
  free(buf);
  return size;
}

void controllerInitialization() {
  if (calibrated) return;
  if (DEBUG) Serial.println("Beginning calibration");

  // Calibrate ESCs and wait for vibrations to stop
  if (DEBUG) Serial.println("ESCs security detachement and arming.");
  driver.calibrate();

  // Wait for vibration to end 6.5s
  delay(6500);

  // Calibrate gyroscope on assumed leveled terrain
  if (DEBUG) Serial.println("Calibration of gyroscope.");
  gyro.calibrateGyro();
  if (DEBUG) Serial.println("Completed.");
  digitalWrite(LED_RED, LOW);
  calibrated = true;

  int throttle = radio.getChannel(THROTTLE);
  int swb = radio.getChannel(SWB);
  // Security measures to avoid risky situations 
  while (swb > 1500 && throttle > 1020) {
    swb = radio.getChannel(SWB);
    throttle = radio.getChannel(THROTTLE);
    digitalWrite(LED_RED, HIGH);
    delay(1000);
    digitalWrite(LED_RED, LOW);
    delay(1000);
  }
}

void execController() {
  // Get control signals
  float throttle = (float) radio.getChannel(THROTTLE);
  int roll = radio.getChannel(ROLL);
  int pitch = radio.getChannel(PITCH);
  int yaw = radio.getChannel(YAW);

  // Get actual state (angles and angle rates)
  float angle[2];
  gyro.kalmanAngle(angle);
  float angleRates[3];
  gyro.accelRates(angleRates);

  // Convert control signals to angles
  float froll = driver.anglePID((float) map(roll, 1000, 2000, -45, 45) - angle[0], 0);
  float fpitch = driver.anglePID((float) map(pitch, 1000, 2000, -45, 45) - angle[1], 1);
  float fyaw = driver.ratePID((float) map(yaw, 1000, 2000, -45, 45) - angleRates[2]);
  // Compute PWMs
  if (DEBUG) Serial.printf("Compute PWMs with throttle %f, roll %f, pitch %f, yaw %f\n", throttle, froll, fpitch, fyaw);
  driver.motorPWMs((float) throttle, froll, fpitch, fyaw);
  if (DEBUG) Serial.println("PWM calculated.");
  // Apply PWMs
  if (throttle > 1050) {
    if (DEBUG) Serial.println("Applying pwm.");
    driver.apply();
    if (DEBUG) Serial.println("PWM applied.");
  } else driver.applyZero();
}