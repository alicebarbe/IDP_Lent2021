#define _USE_MATH_DEFINES
#include <webots/Motor.hpp>
#include "CommunicationClient.hpp"
#include "Movement.hpp"
#include "SimulationParameters.hpp"
#include <tuple>
#include <algorithm>
#include <math.h>
#include <cstdlib>
#include <cmath>

using namespace std;
using namespace webots;

const PIDGains RotationalPIDGains{ 0.1, 0, 0, 0.5 };
const double turn_only_thresh = 10;

const PIDGains ForwardPIDGains{ 100, 0, 0, 0.01 };

tuple<double, double> moveToPosition(tuple<double, double>position, 
                                      tuple<double, double> current_position,
                                        double current_bearing) {

  double target_bearing = atan2(get<1>(position) - get<1>(current_position),
                                get<0>(position) - get<0>(current_position)) * 180 / M_PI;
  if (target_bearing < 0) {
    target_bearing += 360;
  }

  double turning_speed = turnToBearing(target_bearing, current_bearing);
  double forward_speed = 0.0;

  if (abs(getBearingDifference(current_bearing, target_bearing)) < 10) {
    double distance_to_target = pow(get<0>(position) - get<0>(current_position), 2)
      + pow(get<1>(position) - get<1>(current_position), 2);

    forward_speed = clamp(getPIDOutput(distance_to_target, ForwardPIDGains), - MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
  }

  double left_motor_speed = clamp(forward_speed + turning_speed, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
  double right_motor_speed = clamp(forward_speed - turning_speed, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
  return tuple<double, double>(left_motor_speed, right_motor_speed);
}

double turnToBearing(double bearing, double current_bearing) {
  double error = getBearingDifference(current_bearing, bearing);
  double turn_speed = clamp(getPIDOutput(error, RotationalPIDGains), - MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
  return turn_speed;
}

double getBearingDifference(double bearing_one, double bearing_two) {
  double diff = bearing_two - bearing_one;
  if (diff > 180) {
    diff -= 360.0;
  }
  else if (diff <= -180) {
    diff += 360;
  }
  return diff;
}

double getPIDOutput(double error, PIDGains gains) {
  return (abs(error) > gains.moveThresh) ? error * gains.kp : 0;
}

// just copied from the webots docs
double getBearing(const double* vector) {
  double rad = atan2(vector[0], vector[2]);
  double bearing = rad / M_PI * 180.0;
  if (bearing < 0.0)
    bearing = bearing + 360.0;
  return bearing;
}