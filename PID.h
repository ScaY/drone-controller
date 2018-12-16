#ifndef PID_h
#define PID_h

#define KP (6.50)
#define KI (0.001)
#define KD (10.00)
#define MAX_PID (100)

struct PIDDrone {
  double frontLeft;
  double frontRight;
  double rearLeft;
  double rearRight;
};

struct PIDResult {
  double pid;
  float integrate;
  double error;
};

class PID {

  private:
    PIDResult computePid(float angle, float refAngle, double pidIntegrate, float previousError);

    PIDResult computePidPitch(float angle);
    PIDResult computePidRoll(float angle);
    PIDResult computePidYaw(float angle);

  public:
    float refAnglePitch = 0;
    float refAngleRoll = 0;
    float refAngleYaw = 0;

    double integratePitch = 0;
    double integrateRoll = 0;
    double integrateYaw = 0;

    double errorPitch = 0;
    double errorRoll = 0;
    double errorYaw = 0;

    PID();
    PIDDrone computePidDrone(float anglePitch, float angleRoll, float angleYaw);
};

#endif
