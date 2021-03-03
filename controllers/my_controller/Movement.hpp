#pragma once

#include <webots/Motor.hpp>
#include <tuple>

struct PIDGains {
  double kp;
  double ki;
  double kd;
  double moveThresh;
};

std::tuple<double, double> moveToPosition(std::tuple<double, double> position, std::tuple<double, double> current_position, double current_bearing);
double turnToBearing(double bearing, double current_bearing);

//utility functions
double getBearingDifference(double bearing_one, double bearing_two);
double getPIDOutput(double error, PIDGains gains);
double getBearing(const double* vector);