#include <webots/Motor.hpp>
#include "CommunicationClient.hpp"
#include "Movement.hpp"
#include <tuple>
#include <algorithm>

using namespace std;
using namespace webots;

const PIDGains RotationalPIDGains{ 0.1, 0, 0, 3 };

void turnToBearing(tuple<Motor*, Motor*> motors, double bearing, double current_bearing) {
  double error = getBearingDifference(current_bearing, bearing);
  double turn_speed = clamp(getPIDOutput(error, RotationalPIDGains), -1, 1);

  tuple<double, double> motor_speeds = tuple<double, double>(0, 0); // get from motors.cpp function
  //update motor speeds
  get<0>(motor_speeds) += turn_speed;
  get<1>(motor_speeds) -= turn_speed;
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