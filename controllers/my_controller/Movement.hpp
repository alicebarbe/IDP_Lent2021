#pragma once

#include <webots/Motor.hpp>
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
bool hasReachedPosition();
std::tuple<double, double> moveToPosition(std::tuple<double, double> currentPosition, const double* currentBearing);
double turnToBearing(double bearing, double currentBearing);

//utility functions
double getBearingDifference(double bearingOne, double bearingTwo);
double getCompassBearing(const double* vector);
double getBearing(std::tuple<double, double> vector);


double getPIDOutput(double error, PIDGains gains, PIDState state);
