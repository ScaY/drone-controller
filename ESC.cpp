#include "ESC.h"
#include "Arduino.h"

ESC::ESC() {

}

void ESC::init() {

  // set up pin mode
  escWhiteRight.attach(ESC_WHITE_RIGHT, MIN_PULSE_RATE, MAX_PULSE_RATE);
  escWhiteLeft.attach(ESC_WHITE_LEFT, MIN_PULSE_RATE, MAX_PULSE_RATE);
  escRedRight.attach(ESC_RED_RIGHT, MIN_PULSE_RATE, MAX_PULSE_RATE);
  escRedLeft.attach(ESC_RED_LEFT, MIN_PULSE_RATE, MAX_PULSE_RATE);


  // init motor to min
  Serial.println("Send min to ESC");
  escWhiteRight.writeMicroseconds(MIN_PULSE_RATE);
  escRedRight.writeMicroseconds(MIN_PULSE_RATE);
  escWhiteLeft.writeMicroseconds(MIN_PULSE_RATE);
  escRedLeft.writeMicroseconds(MIN_PULSE_RATE);
  delay(2000);

  // start motor
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  Serial.println("Start motor");
  delay(2000);

  // init motor to max
  Serial.println("Send max to ESC");
  escWhiteRight.writeMicroseconds(MAX_PULSE_RATE);
  escRedRight.writeMicroseconds(MAX_PULSE_RATE);
  escWhiteLeft.writeMicroseconds(MAX_PULSE_RATE);
  escRedLeft.writeMicroseconds(MAX_PULSE_RATE);
  delay(2000);

}

void ESC::stopMotor() {
  escWhiteRight.writeMicroseconds(MIN_PULSE_RATE);
  escRedRight.writeMicroseconds(MIN_PULSE_RATE);
  escWhiteLeft.writeMicroseconds(MIN_PULSE_RATE);
  escRedLeft.writeMicroseconds(MIN_PULSE_RATE);
}
