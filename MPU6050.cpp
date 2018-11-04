#include "MPU6050.h"
#include "Arduino.h"

MPU6050::MPU6050() {

}

void MPU6050::init() {
  Serial.print("MPU initialisation... ");
  Wire.endTransmission();

  Wire.beginTransmission(GYRO_ADRESS);                        //Start communication with the MPU-6050.
  Wire.write(0x6B);                                            //We want to write to the PWR_MGMT_1 register (6B hex).
  Wire.write(0x00);                                            //Set the register bits as 00000000 to activate the gyro.
  Wire.endTransmission();                                      //End the transmission with the gyro.

  Wire.beginTransmission(GYRO_ADRESS);                        //Start communication with the MPU-6050.
  Wire.write(0x1B);                                            //We want to write to the GYRO_CONFIG register (1B hex).
  Wire.write(0x08);                                            //Set the register bits as 00001000 (500dps full scale).
  Wire.endTransmission();                                      //End the transmission with the gyro.

  Wire.beginTransmission(GYRO_ADRESS);                        //Start communication with the MPU-6050.
  Wire.write(0x1C);                                            //We want to write to the ACCEL_CONFIG register (1A hex).
  Wire.write(0x10);                                            //Set the register bits as 00010000 (+/- 8g full scale range).
  Wire.endTransmission();                                      //End the transmission with the gyro.

  Wire.beginTransmission(GYRO_ADRESS);                        //Start communication with the MPU-6050.
  Wire.write(0x1A);                                            //We want to write to the CONFIG register (1A hex).
  Wire.write(0x03);                                            //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz).
  Wire.endTransmission();                                      //End the transmission with the gyro.

  Serial.println("OK");

}

AccelRaw MPU6050::readAccelRaw() {

  Wire.beginTransmission(GYRO_ADRESS);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission();
  Wire.requestFrom(GYRO_ADRESS, 14);

  while (Wire.available() < 14);                               //Wait until all the bytes are received

  AccelRaw accelRaw;

  // Invert the accelometer data to be aligne with the gyro
  accelRaw.accY = Wire.read() << 8 | Wire.read();
  accelRaw.accX = Wire.read() << 8 | Wire.read();
  accelRaw.accZ = Wire.read() << 8 | Wire.read();
  accelRaw.temperature = Wire.read() << 8 | Wire.read();
  accelRaw.gyroRoll = Wire.read() << 8 | Wire.read();
  accelRaw.gyroPitch = Wire.read() << 8 | Wire.read();
  accelRaw.gyroYaw = Wire.read() << 8 | Wire.read();

  accelRaw.gyroRoll -= this->gyroRollCal;
  accelRaw.gyroPitch -= this->gyroPitchCal;
  accelRaw.gyroYaw -= this->gyroYawCal;

  //accelRaw.accY -= this->accYCal;
  //accelRaw.accX -= this->accXCal;

  accelRaw.accY *= -1;
  //accelRaw.accX *= -1;

  return accelRaw;
}

void MPU6050::calibrateGyro() {

  Serial.print("MPU gyro calibration... ");
  double gyroRollCalSum = 0;
  double gyroPitchCalSum = 0;
  double gyroYawCalSum = 0;
  double accXCalSum = 0;
  double accYCalSum = 0;

  int nbCal = 1000;
  int i = 0;

  for (i = 0; i < nbCal;  i++) {
    AccelRaw accelRaw = this->readAccelRaw();
    gyroRollCalSum += accelRaw.gyroRoll;
    gyroPitchCalSum += accelRaw.gyroPitch;
    gyroYawCalSum += accelRaw.gyroYaw;
    delay(3); // 250 Hz
  }

  this->gyroRollCal = gyroRollCalSum / nbCal;
  this->gyroPitchCal = gyroPitchCalSum / nbCal;
  this->gyroYawCal = gyroYawCalSum / nbCal;



  Serial.print("Gyro calibration roll ");
  Serial.print(this->gyroRollCal);
  Serial.print(" pitch ");
  Serial.println(this->gyroPitchCal);

  Serial.println("OK");
}

AccelAngles MPU6050::computeAngles() {

  AccelRaw accelRaw = readAccelRaw();

  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz * 65.5)
  float convertGyroData = 0.0000611;
  accelAngles.pitch += accelRaw.gyroPitch * 0.0000611;
  accelAngles.roll += accelRaw.gyroRoll * 0.0000611;

  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians and not degrees.
  accelAngles.pitch -=  accelAngles.roll * sin((float)accelRaw.gyroYaw * 0.000001066);                  //If the IMU has yawed transfer the roll angle to the pitch angel.
  accelAngles.roll +=  accelAngles.pitch * sin((float)accelRaw.gyroYaw * 0.000001066);                  //If the IMU has yawed transfer the pitch angle to the roll angel.


  //Accelerometer angle calculations
  float accTotalVector = sqrt((accelRaw.accX * accelRaw.accX) + (accelRaw.accY * accelRaw.accY) + (accelRaw.accZ * accelRaw.accZ));    //Calculate the total accelerometer vector.
  float anglePitchAcc = 0;
  float angleRollAcc = 0;


  float scaleFactor = 1 / (4.096 / 180);
  if (abs(accelRaw.accY) < accTotalVector) {                                             //Prevent the asin function to produce a NaN.
    anglePitchAcc = asin((float)accelRaw.accY / accTotalVector) * scaleFactor;              //Calculate the pitch angle.
  }
  if (abs(accelRaw.accX) < accTotalVector) {                                             //Prevent the asin function to produce a NaN.
    angleRollAcc = asin((float)accelRaw.accX / accTotalVector) * scaleFactor;               //Calculate the roll angle.
  }

  // Serial.print( anglePitchAcc);
  // Serial.print(" \t");

  if (this->setupGyro) {

    accelAngles.pitch = accelAngles.pitch * 0.9996 + anglePitchAcc * 0.0004;                   //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
    accelAngles.roll = accelAngles.roll * 0.9996 + angleRollAcc * 0.0004;                     //Correct the drift of the gyro roll angle with the accelerometer roll angle.

  } else {
    accelAngles.pitch = anglePitchAcc;
    accelAngles.roll = angleRollAcc;
    this->setupGyro = true;
  }


  Serial.print(accelAngles.pitch);
  Serial.print(" \t");
  Serial.print( accelAngles.roll);
  Serial.print(" \t");


  return accelAngles;
}

void MPU6050::printAngles() {

  Serial.print("Pitch angle: ");
  Serial.print(this->accelAngles.pitch);
  Serial.print("\tRoll angle: ");
  Serial.println(this->accelAngles.roll);
}
