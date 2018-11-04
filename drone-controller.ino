
#include <Wire.h>
#include <Servo.h>
#include "MPU6050.h"
#include "FSI6.h"
#include "ESC.h"
#include "PID.h"

//#define ADXL345 (0x53)             // Device address as specified in data sheet //ADXL345 accelerometer
#define GYRO (0x68)               // Gyro address
#define HMC5883 (0x1E)
#define ACC (0x68)
#define GYRO_DATA (0x1B)        // Gyro data first address
#define DATAX0  (0x32)         //X-Axis Data 0
#define G_DATAX0 (0x1D)
#define G_SMPLRT_DIV 0x15
#define G_DLPF_FS 0x16
#define G_INT_CFG 0x17
#define G_PWR_MGM 0x3E
#define ACC_UNIT (8,192.0)

MPU6050 mpu;
FSI6 fsi;
ESC esc;
PID pid;

float timeDevice, timePrev, elapsedTime, loopTimer;

int targetStopMotorValue = 1080;

float refAngleX = 0, refAngleY = 0;
float throttle = 1100;

float pidResult[5];
float error = 0, previousErrorX = 0, previousErrorY = 0, pidIntegrateX = 0, pidIntegrateY = 0, pidProportional = 0, pidDerivative = 0, pwmLeft, pwmRight;

int minPulseRate = 1100;
int maxPulseRate = 1900;

Servo escWhiteRight;
Servo escWhiteLeft;
Servo escRedRight;
Servo escRedLeft;

int relayPin = 7;         // Relay
int ledPidWhitePin = 6;   // led to warn that a pid has been set

int nbSignalLost = 0;
int nbIterations = 0;
int nbMaxIterations = 100;


unsigned long timer[4];
byte lastChannel[4];
int input[4];

void setup() {

  Wire.begin();
  Serial.begin(115200);

  mpu.init();
  fsi.init();

  timeDevice = micros();
  loopTimer = micros();

  // Led pin
  pinMode(ledPidWhitePin, OUTPUT);

  mpu.calibrateGyro();

  PCICR |= ( 1 << PCIE0 );
  PCMSK0 |= ( 1 << PCINT1);
  PCMSK0 |= ( 1 << PCINT2);
  PCMSK0 |= ( 1 << PCINT3);
  PCMSK0 |= ( 1 << PCINT4);

  esc.init();


  AccelAngles accelAngles = mpu.computeAngles();
  Serial.print("Desired angle \t");
  mpu.printAngles();
  refAngleX = accelAngles.pitch;

  pid.refAnglePitch = accelAngles.pitch;
  pid.refAngleRoll = accelAngles.roll;
  pid.refAngleYaw = accelAngles.yaw;

  Serial.print("P: ");
  Serial.print(KP);
  Serial.print(" I: ");
  Serial.print(KI);
  Serial.print(" D: ");
  Serial.println(KD);

}

ISR(PCINT0_vect) {


  long timerValue = micros();
  if (lastChannel[0] == 0 && PINB & B00000010) {
    lastChannel[0] = 1;
    timer[0] = timerValue;
  } else if (lastChannel[0] == 1 && !(PINB & B00000010)) {
    lastChannel[0] = 0;
    input[0] = timerValue - timer[0];
  }

  if (lastChannel[1] == 0 && PINB & B00000100) {
    lastChannel[1] = 1;
    timer[1] = timerValue;
  } else if (lastChannel[1] == 1 && !(PINB & B00000100)) {
    lastChannel[1] = 0;
    input[1] = timerValue - timer[1];
  }

  if (lastChannel[2] == 0 && PINB & B00001000) {
    lastChannel[2] = 1;
    timer[2] = timerValue;
  } else if (lastChannel[2] == 1 && !(PINB & B00001000)) {
    lastChannel[2] = 0;
    input[2] = timerValue - timer[2];
  }


  if (lastChannel[3] == 0 && PINB & B00010000) {
    lastChannel[3] = 1;
    timer[3] = timerValue;
  } else if (lastChannel[3] == 1 && !(PINB & B00010000)) {
    lastChannel[3] = 0;
    input[3] = timerValue - timer[3];
  }
}


void printInterrupt() {
  Serial.print(input[0]);
  Serial.print(" - ");
  Serial.print(input[1]);
  Serial.print(" - ");
  Serial.print(input[2]);
  Serial.print(" - ");
  Serial.print(input[3]);
}

void loop() {

  //printInterrupt();


  if (input[1] < targetStopMotorValue && input[2] < targetStopMotorValue) {
    // Stop the motor
    throttle = 0;
  } else {
    // convert the RC value into the motor value
    throttle = map(input[2], 1000, 1980, minPulseRate - 50, maxPulseRate);
  }

  // compute the angles (pitch and roll)
  AccelAngles accelAngles = mpu.computeAngles();


  PIDDrone pidDrone = pid.computePidDrone(accelAngles.pitch, accelAngles.roll, accelAngles.yaw);
  
  esc.frontLeft.writeMicroseconds(throttle + pidDrone.frontLeft);
  esc.frontRight.writeMicroseconds(throttle + pidDrone.frontRight);
  esc.rearLeft.writeMicroseconds(throttle + pidDrone.rearLeft);
  esc.rearRight.writeMicroseconds(throttle + pidDrone.rearRight);
  
  if (abs(pidResult[4]) > 10) {
    digitalWrite(ledPidWhitePin, HIGH);
  } else {
    digitalWrite(ledPidWhitePin, LOW);
  }


  //We wait until 4000us are passed.
  float delta = micros() - loopTimer;
  Serial.print("\t");
  Serial.println(delta);
  while (micros() - loopTimer < 4000);
  loopTimer = micros();

}

/**
   Compute the PID to stabilize the drone
*/
/*
void computePid(float totalAngleX, float pidIntegrate, float previousError, float elapsedTime, boolean logResult ) {

  error = totalAngleX - refAngleX;

  pidProportional = KP * error;
  if (error < 3 && error > -3) {
    pidIntegrate = pidIntegrate + KI * error;
  }
  //pidDerivative = KD * ( (error - previousError) / elapsedTime);
  pidDerivative = KD * (error - previousError);

  pid = pidProportional + pidIntegrate + pidDerivative;
  pid = constrain(pid, -MAX_PID, MAX_PID);

  pwmLeft = throttle + pid;
  pwmRight = throttle - pid;
  pwmLeft = constrain(pwmLeft, minPulseRate, maxPulseRate);
  pwmRight = constrain(pwmRight, minPulseRate, maxPulseRate);

  pidResult[0] = pwmRight;
  pidResult[1] = pwmLeft;
  pidResult[2] = error;
  pidResult[3] = pidIntegrate;
  pidResult[4] = pid;
}*/
