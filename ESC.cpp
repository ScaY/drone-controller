#include "ESC.h"
#include "Arduino.h"

ESC::ESC() {

}

void ESC::init() {

  // set up pin mode
  frontLeft.attach(ESC_FRONT_LEFT, MIN_PULSE_RATE, MAX_PULSE_RATE);
  frontRight.attach(ESC_FRONT_RIGHT, MIN_PULSE_RATE, MAX_PULSE_RATE);
  rearRight.attach(ESC_REAR_RIGHT, MIN_PULSE_RATE, MAX_PULSE_RATE);
  rearLeft.attach(ESC_REAR_LEFT, MIN_PULSE_RATE, MAX_PULSE_RATE);


  // init motor to min
  Serial.println("Send min to ESC");
  frontRight.writeMicroseconds(MIN_PULSE_RATE);
  frontLeft.writeMicroseconds(MIN_PULSE_RATE);
  rearRight.writeMicroseconds(MIN_PULSE_RATE);
  rearLeft.writeMicroseconds(MIN_PULSE_RATE);
  delay(2000);

  // start motor
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  Serial.println("Start motor");
  delay(2000);

  // init motor to max
  Serial.println("Send max to ESC");
  frontRight.writeMicroseconds(MAX_PULSE_RATE);
  frontLeft.writeMicroseconds(MAX_PULSE_RATE);
  rearRight.writeMicroseconds(MAX_PULSE_RATE);
  rearLeft.writeMicroseconds(MAX_PULSE_RATE);
  delay(2000);

}

void ESC::stopMotor() {
  frontRight.writeMicroseconds(MIN_PULSE_RATE);
  frontLeft.writeMicroseconds(MIN_PULSE_RATE);
  rearRight.writeMicroseconds(MIN_PULSE_RATE);
  rearLeft.writeMicroseconds(MIN_PULSE_RATE);
}
