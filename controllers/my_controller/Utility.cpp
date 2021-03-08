#define _USE_MATH_DEFINES
#include <tuple>
#include <algorithm>
#include <math.h>
#include <cstdlib>
#include <cmath>

#include "Utility.hpp"

using namespace std;

/*
* This funcion only works for the compass - it gives the
* bearing of the vector in the direction the robot faces
*/
double getCompassBearing(const double* vector) {
  double rad = atan2(vector[0], vector[2]);
  double bearing = rad / M_PI * 180.0;
  if (bearing < 0.0)
    bearing = bearing + 360.0;
  return bearing;
}

double getBearing(tuple<double, double> vector) {
  double rad = atan2(get<1>(vector), get<0>(vector));
  double bearing = rad / M_PI * 180.0;
  if (bearing < 0.0)
    bearing = bearing + 360.0;
  return bearing;
}

double getBearingDifference(double bearingOne, double bearingTwo) {
  double diff = bearingTwo - bearingOne;
  if (diff > 180) {
    diff -= 360.0;
  }
  else if (diff <= -180) {
    diff += 360;
  }
  return diff;
}

tuple<double, double> rotateVector(const tuple<double, double> vector, double angle) {
  double radAngle = angle * M_PI / 180.0;
  double rotatedX = get<0>(vector) * cos(radAngle) - get<1>(vector) * sin(radAngle);
  double rotatedZ = get<0>(vector) * sin(radAngle) + get<1>(vector) * cos(radAngle);
  return tuple<double, double>(rotatedX, rotatedZ);
}

double getWallDistance(const double* robot_pos, double angle, const tuple<double, double> sensorDisplacement) {
  double boundXPos, boundXNeg, boundZPos, boundZNeg;
  double radAngle = angle * M_PI / 180.0;
  tuple<double, double> rotatedSensorDisp = rotateVector(sensorDisplacement, angle);

  boundXPos = (1.2 - robot_pos[0] - get<0>(rotatedSensorDisp)) / cos(radAngle);
  boundXNeg = (-1.2 - robot_pos[0] - get<0>(rotatedSensorDisp)) / cos(radAngle);

  boundZPos = (1.2 - robot_pos[2] - get<1>(rotatedSensorDisp)) / sin(radAngle);
  boundZNeg = (-1.2 - robot_pos[2] - get<1>(rotatedSensorDisp)) / sin(radAngle);

  return min(max(boundXPos, boundXNeg), max(boundZNeg, boundZPos));
}