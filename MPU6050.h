#ifndef MPU6050_h
#define MPU6050_h

#include "Arduino.h"
#include <Wire.h>

#define PWR_MGMT_1 0x6B
#define SIGNAL_PATH_RESET 0x68
#define ACCEL_XOUT_H 0x3B
#define GYRO_ADRESS 0x68

struct AccelRaw {
  long accX;
  long accY;
  long accZ;
  int16_t temperature;
  long gyroRoll;
  long gyroPitch;
  long gyroYaw;
};

struct AccelAngles {
  double pitch;
  double roll;
};

class MPU6050 {
  public:
    double accPitchCal = 0;
    double accRollCal = 0;
    double gyroRollCal = 0;
    double gyroPitchCal = 0;
    double gyroYawCal = 0;
    double accXCal = 0;
    double accYCal = 0;
    AccelAngles accelAngles;
    boolean setupGyro = false; //false;

    MPU6050();
    void init();
    AccelRaw readAccelRaw();
    AccelAngles computeAngles();
    void printAngles();
    void calibrateGyro();
};

#endif
