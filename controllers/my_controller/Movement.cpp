#include <webots/Motor.hpp>
#include "CommunicationClient.hpp"
#include "Movement.hpp"
#include <tuple>
#include <algorithm>
#include <math.h>
#include <cstdlib>

using namespace std;
using namespace webots;

const PIDGains RotationalPIDGains{ 0.1, 0, 0, 1 };
const double turn_only_thresh = 10;

const PIDGains ForwardPIDGains{ 10, 0, 0, 0.01 };

void moveToPosition(tuple<Motor*, Motor*> motors,
                    tuple<double, double>position, 
                    tuple<double, double> current_position,
                    double current_bearing) {

  double target_bearing = atan2(get<1>(position) - get<1>(current_position),
                                get<0>(position) - get<0>(current_position));
  if (target_bearing < 0) {
    target_bearing += 360;
  }

  turnToBearing(motors, target_bearing, current_bearing);

  if (abs(getBearingDifference(current_bearing, target_bearing)) < 10) {
    double distance_to_target = pow(get<0>(position) - get<0>(current_position), 2)
      + pow(get<1>(position) - get<1>(current_position), 2);

    double speed = clamp(getPIDOutput(distance_to_target, ForwardPIDGains), -1.0, 1.0);
    //run a set motor speed function
  }

}

void turnToBearing(tuple<Motor*, Motor*> motors, double bearing, double current_bearing) {
  double error = getBearingDifference(current_bearing, bearing);
  double turn_speed = clamp(getPIDOutput(error, RotationalPIDGains), -1.0, 1.0);
  //run a set motor speed function
}

double getBearingDifference(double bearing_one, double bearing_two) {
  double diff = bearing_two - bearing_one;
  if (diff > 180) {
    diff -= 360.0;
  }
  else if (diff < -180) {
    diff += 360;
  }
  return diff;
}

double getPIDOutput(double error, PIDGains gains) {
  return (error > gains.moveThresh) ? error * gains.kp : 0;
}

// just copied from the webots docs
// this is only here temporarily - should go with the compass
double get_bearing_in_degrees(const double* vector) {
  double rad = atan2(vector[0], vector[2]);
  double bearing = (rad - 1.5708) / M_PI * 180.0;
  if (bearing < 0.0)
    bearing = bearing + 360.0;
  return bearing;
}