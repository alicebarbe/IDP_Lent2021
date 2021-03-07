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
void tweakBlockDistanceFromMeasurement(std::tuple<double, double> robotPosition, std::tuple<double, double> sensorDisplacement, std::tuple<double, double> frontOfRobotDisplacement, const double* currentBearingVector, double distance);
bool hasReachedPosition();
bool hasFinishedTurning();
std::tuple<double, double> moveToPosition(std::tuple<double, double> currentPosition, const double* currentBearing);
double turnToBearing(double bearing, double currentBearing);


//utility functions
double getBearingDifference(double bearingOne, double bearingTwo);
double getCompassBearing(const double* vector);
double getBearing(std::tuple<double, double> vector);

double getPIDOutput(double error, PIDGains gains, PIDState state);

std::tuple<double, double> rotateVector(const std::tuple<double, double> vector, double angle);
