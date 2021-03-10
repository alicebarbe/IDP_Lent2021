#define _USE_MATH_DEFINES
#include <tuple>
#include <algorithm>
#include <math.h>
#include <cstdlib>
#include <cmath>

#include "Utility.hpp"
#include "Coordinate.hpp"
#include "SimulationParameters.hpp"

using namespace std;

/*
* This funcion only works for the compass - it gives the
* bearing of the vector in the direction the robot faces
*/
double getCompassBearing(const double* vector) {
  double rad = atan2(vector[2], -vector[0]);
  double bearing = rad * RAD_TO_DEG;
  if (bearing < 0.0)
    bearing = bearing + 360.0;
  return bearing;
}

double getBearing(coordinate vector) {
  double rad = atan2(vector.z, vector.x);
  double bearing = rad * RAD_TO_DEG;
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

double constrainBearing(double bearing) {
  return bearing > 360 ? constrainBearing(bearing - 360) : bearing;
}

coordinate rotateVector(const coordinate vector, double angle) {
  double radAngle = angle * DEG_TO_RAD;
  double rotatedX = vector.x * cos(radAngle) - vector.z * sin(radAngle);
  double rotatedZ = vector.x * sin(radAngle) + vector.z * cos(radAngle);
  return coordinate(rotatedX, rotatedZ);
}

double getWallDistance(const coordinate robotPos, double angle) {
  double boundXPos, boundXNeg, boundZPos, boundZNeg;
  double radAngle = angle * DEG_TO_RAD;
  coordinate rotatedSensorDisp = rotateVector(distanceSensorDisplacement, angle);

  boundXPos = (ARENA_X_MAX - robotPos.x - rotatedSensorDisp.x) / cos(radAngle);
  boundXNeg = (ARENA_X_MIN - robotPos.x - rotatedSensorDisp.x) / cos(radAngle);

  boundZPos = (ARENA_Z_MAX - robotPos.z - rotatedSensorDisp.z) / sin(radAngle);
  boundZNeg = (ARENA_Z_MIN - robotPos.z - rotatedSensorDisp.z) / sin(radAngle);

  return min(max(boundXPos, boundXNeg), max(boundZNeg, boundZPos));
}