#pragma once

#include <tuple>

struct PIDGains {
  double kp;
  double ki;
  double kd;
  double moveThresh;
  double integralThresh;
};

struct PIDState {
  double lastError;
  double integral;
};

void updateTargetPosition(std::tuple<double, double> newTarget);
void tweakBlockDistanceFromMeasurement(std::tuple<double, double> robotPosition, const double* currentBearingVector, double distance);
bool hasReachedPosition();
bool hasFinishedTurning();
std::tuple<double, double> moveToPosition(std::tuple<double, double> currentPosition, const double* currentBearing);
double turnToBearing(double bearing, double currentBearing);
double getPIDOutput(double error, PIDGains gains, PIDState state);
std::tuple<double, double> getPositionInfrontOfBlock(std::tuple<double, double> blockPosition, std::tuple<double, double> robotPosition);
std::tuple<double, double> getBlockPosition(std::tuple<double, double> afterLastJump, std::tuple<double, double> beforeJump, bool lastJumpWasFall, bool jumpWasFall,
  const double* robot_pos, const double sensorBeamAngle);
