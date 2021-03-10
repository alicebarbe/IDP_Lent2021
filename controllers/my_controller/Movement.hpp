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

// Updating target functions
void updateTargetPosition(coordinate newTarget, bool reverse=false);
void updateTargetBearing(double newBearing);
void updateTargetDistance(coordinate newTarget, bool reverse=true);
void tweakTargetDistanceFromMeasurement(coordinate robotPosition, const double* currentBearingVector, double distance);

// flag retrieval functions used to monitor control behavior
bool hasReachedPosition();
bool canUseDistanceSensor();
bool hasReachedTargetBearing();
coordinate getTargetPosition();

// Control loop functions to be called in a main loop
std::tuple<double, double> updateRotationControlLoop(const double* currentBearingVector);
std::tuple<double, double> updatePositionalControlLoop(coordinate currentPosition, const double* currentBearingVector);
double getBearingCorrection(double bearing, double currentBearing);

// Positioning functions - these should probably go elsewhere
coordinate getPositionAroundBlock(coordinate blockPosition, coordinate robotPosition, coordinate displacement);
coordinate getBlockPosition(std::tuple<double, double> afterLastJump, std::tuple<double, double> beforeJump, bool lastJumpWasFall, bool jumpWasFall,
  coordinate robotPosition, const double sensorBeamAngle);
double getWallDistance(const coordinate robotPos, double angle);
