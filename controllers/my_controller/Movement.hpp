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
void updateTargetPosition(coordinate newTarget, bool reverse = false);
void updateTargetBearing(double newBearing);
void updateTargetDistance(coordinate newTarget, bool reverse = true);
bool tweakTargetDistanceFromMeasurement(coordinate robotPosition, const double* currentBearingVector, double distance, double lostThreshold);

// flag retrieval functions used to monitor control behavior
bool hasReachedPosition();
bool canUseDistanceSensor();
bool hasReachedTargetBearing();
coordinate getTargetPosition();
bool isMaintainingTargetBearing();

// Control loop functions to be called in a main loop
std::tuple<double, double> updateRotationControlLoop(const double* currentBearingVector);
std::tuple<double, double> updatePositionalControlLoop(coordinate currentPosition, const double* currentBearingVector);
double getBearingCorrection(double bearing, double currentBearing);

// Positioning functions - these should probably go elsewhere
coordinate getPositionAroundBlock(coordinate blockPosition, coordinate robotPosition, coordinate displacement);
coordinate getBlockPosition(std::tuple<double, double> afterLastJump, std::tuple<double, double> beforeJump, bool lastJumpWasFall, bool jumpWasFall,
  coordinate robotPosition, const double sensorBeamAngle);
coordinate getBlockPositionFromAngleAndDistance(coordinate robotPosition, double blockAvgDistance, double blockAvgAngle);
double getWallDistance(const coordinate robotPos, double angle);
double getExpectedDistanceOfBlock(coordinate robotPosition, const double* currentBearingVector);
coordinate getBlockPositionInGrabber(coordinate robotPosition, double bearing);
