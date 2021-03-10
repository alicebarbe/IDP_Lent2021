#pragma once

#include <tuple>
#include "Coordinate.hpp"

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

void updateTargetPosition(coordinate newTarget);
void updateTargetBearing(double newBearing);
void tweakBlockDistanceFromMeasurement(coordinate robotPosition, const double* currentBearingVector, double distance);
bool hasReachedPosition();
bool hasFinishedTurning();
bool hasReachedTargetBearing();
std::tuple<double, double> turnToTargetBearing(const double* currentBearingVector);
std::tuple<double, double> moveToPosition(coordinate currentPosition, const double* currentBearingVector);
double turnToBearing(double bearing, double currentBearing);
double getPIDOutput(double error, PIDGains gains, PIDState state);
coordinate getPositionInfrontOfBlock(coordinate blockPosition, coordinate robotPosition);
coordinate getBlockPosition(std::tuple<double, double> afterLastJump, std::tuple<double, double> beforeJump, bool lastJumpWasFall, bool jumpWasFall,
  coordinate robotPosition, const double sensorBeamAngle);
coordinate getTargetPosition();
coordinate getPositionBeyondBlock(coordinate targetPosition, const double* bearingvector, double distance);
