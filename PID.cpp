#include "PID.h"
#include "Arduino.h"

PID::PID() {

}

/**
 * Comptue the PID for each motor
 */
PIDDrone PID::computePidDrone(float anglePitch, float angleRoll, float angleYaw) {


  PIDResult pidPitch = this->computePidPitch(anglePitch);
  PIDResult pidRoll = this->computePidRoll(angleRoll);
  PIDResult pidYaw = this->computePidYaw(angleYaw);

  PIDDrone pidDrone;
  pidDrone.frontLeft = -pidPitch.pid + pidRoll.pid;
  pidDrone.frontRight = -pidPitch.pid - pidRoll.pid;
  pidDrone.rearLeft = pidPitch.pid + pidRoll.pid;
  pidDrone.rearRight = pidPitch.pid - pidRoll.pid;

  return pidDrone;
}


PIDResult PID::computePidPitch(float anglePitch) {
  PIDResult pid = this->computePid(anglePitch, this->refAnglePitch, this->integratePitch, this->errorPitch);
  this->errorPitch = pid.error;
  this->integratePitch += pid.integrate;

  return pid;
}

PIDResult PID::computePidRoll(float angleRoll) {
  PIDResult pid = this->computePid(angleRoll, this->refAngleRoll, this->integrateRoll, this->errorRoll);
  this->errorRoll = pid.error;
  this->integrateRoll += pid.integrate;

  return pid;
}


PIDResult PID::computePidYaw(float angleYaw) {
  PIDResult pid = this->computePid(angleYaw, this->refAngleYaw, this->integrateYaw, this->errorYaw);
  this->errorYaw = pid.error;
  this->integrateYaw += pid.integrate;

  return pid;
}

/**
 * Compute the PID
 */
PIDResult PID::computePid(float angle, float refAngle, float pidIntegrate, float previousError ) {

  PIDResult pidResult;
  double error = angle - refAngle;
  float pid = 0 , pidProportional = 0, pidDerivative = 0;

  pidProportional = KP * error;
  pidDerivative = KD * (error - previousError);
  pidIntegrate = pidIntegrate + KI * error;

  pid = pidProportional + pidIntegrate + pidDerivative;
  pid = constrain(pid, -MAX_PID, MAX_PID);

  pidResult.pid = pid;
  pidResult.error = error;
  pidResult.integrate = pidIntegrate;

}
