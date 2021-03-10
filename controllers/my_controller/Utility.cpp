#define _USE_MATH_DEFINES
#include <tuple>
#include <algorithm>
#include <math.h>
#include <cstdlib>
#include <cmath>

#include "Utility.hpp"
#include "Coordinate.hpp"
#include "SimulationParameters.hpp"
#include "Movement.hpp"

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
  if (bearing >= 0 && bearing < 360) {
    return bearing;
  }
  else return bearing - 360 * floor(bearing / 360.0);
}

coordinate rotateVector(const coordinate vector, double angle) {
  double radAngle = angle * DEG_TO_RAD;
  double rotatedX = vector.x * cos(radAngle) - vector.z * sin(radAngle);
  double rotatedZ = vector.x * sin(radAngle) + vector.z * cos(radAngle);
  return coordinate(rotatedX, rotatedZ);
}

double getPIDOutput(double error, PIDGains gains, PIDState state) {
  double diff = 0;
  if (gains.kd != 0) {
    diff = (state.lastError - error) * gains.kd;  // assumes a constant timestep
  }
  if (gains.ki != 0) {
    state.integral += (abs(error) < gains.integralThresh) ? gains.ki * error : 0;
  }
  return (abs(error) > gains.moveThresh) ? error * gains.kp + state.integral + diff : 0;
}