#ifndef ESC_h
#define ESC_h

#include <Servo.h>
#include "Arduino.h"

#define ESC_FRONT_LEFT 2
#define ESC_FRONT_RIGHT 4
#define ESC_REAR_RIGHT 3
#define ESC_REAR_LEFT 5

#define MIN_PULSE_RATE 1100
#define MAX_PULSE_RATE 1900

#define RELAY_PIN 7

class ESC {
  public:

    ESC();
    void init();
    void stopMotor();
    
    Servo frontLeft;
    Servo frontRight;
    Servo rearRight;
    Servo rearLeft;
};

#endif
