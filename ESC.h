#ifndef ESC_h
#define ESC_h

#include <Servo.h>
#include "Arduino.h"

#define ESC_WHITE_LEFT 2
#define ESC_WHITE_RIGHT 3
#define ESC_RED_LEFT 4
#define ESC_RED_RIGHT 5

#define MIN_PULSE_RATE 1100
#define MAX_PULSE_RATE 1900

#define RELAY_PIN 7

class ESC {
  public:

    ESC();
    void init();
    void stopMotor();

    Servo escWhiteRight;
    Servo escWhiteLeft;
    Servo escRedRight;
    Servo escRedLeft;
};

#endif
