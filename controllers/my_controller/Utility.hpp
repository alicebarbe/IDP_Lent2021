#pragma once

#include <tuple>
#include <vector>

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

double getCompassBearing(const double* vector);
double getBearing(coordinate vector);
double getBearingDifference(double bearingOne, double bearingTwo);
double constrainBearing(double bearing);
coordinate rotateVector(const coordinate vector, double angle);
double getPIDOutput(double error, PIDGains gains, PIDState state);
double distanceBetweenPoints(coordinate pos1, coordinate pos2);
double distanceToTrajectory(coordinate currentPos, coordinate trajectoryPos1, coordinate trajectoryPos2);