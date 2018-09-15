#ifndef MPU6050_h
#define MPU6050_h

#include "Arduino.h"
#include <Wire.h>

#define PWR_MGMT_1 0x6B
#define SIGNAL_PATH_RESET 0x68
#define ACCEL_XOUT_H 0x3B
#define GYRO_ADRESS 0x68

struct AccelRaw {
  int16_t accX;
  int16_t accY;
  int16_t accZ;
  int16_t temperature;
  int16_t gyroRoll;
  int16_t gyroPitch;
  int16_t gyroYaw;
};

struct AccelAngles {
  double pitch;
  double roll;
};

class MPU6050 {
  public:
    double gyroRollCal = 0;
    double gyroPitchCal = 0;
    double gyroYawCal = 0;
    AccelAngles accelAngles;
  
    MPU6050();
    void init();
    AccelRaw readAccelRaw();
    AccelAngles computeAngles();
    void printAngles();
    void calibrateGyro();
};

#endif
