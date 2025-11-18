#include <Arduino.h>

#include "RadioController.h"

void Radio::begin(HardwareSerial& serial, int timer, int pinInput, int pinOutput) {
  ibus.begin(serial, timer, pinInput, pinOutput);
}

int* Radio::readLevers() {
  static int data[6];

  for (int i = 0; i < 6; i++) {
    data[i] = ibus.readChannel(i);
  }
  return data;
}

int Radio::getChannel(int channel) {
  int res = ibus.readChannel(channel);
  if (res < 1000 || channel > 2000) return 1000;
  return res;
}