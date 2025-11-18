#ifndef RADIOCONTROLLER_H
#define RADIOCONTROLLER_H

#include <IBusBM.h>

class Radio {
  private:
  IBusBM ibus;

  public:
  int getChannel(int channel);
  int* readLevers();
  void begin(HardwareSerial& serial, int timer, int pinInput, int pinOutput);
};

#endif