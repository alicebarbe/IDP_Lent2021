#pragma once

#include <webots/Motor.hpp>
#include <tuple>

struct PIDGains {
  double kp;
  double ki;
  double kd;
  double moveThresh;
};

void moveToPosition(std::tuple<webots::Motor*, webots::Motor*> motors, std::tuple<double, double> position);
void turnToBearing(std::tuple<webots::Motor*, webots::Motor*> motors, double bearing);

//utility functions
double getBearingDifference(double bearing_one, double bearing_two);
double getPIDOutput(double setpoint, double current_var, PIDGains gains);