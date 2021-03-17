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

double distanceBetweenPoints(coordinate pos1, coordinate pos2) {
    return sqrt(pow(pos1.x - pos2.x, 2) + pow(pos1.z - pos2.z, 2));
}

double distanceToTrajectory(coordinate currentPos, coordinate trajectoryPos1, coordinate trajectoryPos2) {
    // calculate the distance from currentPos to the line between trajectoryPos1 and trajectoryPos2
    coordinate trajectoryVector = trajectoryPos2 - trajectoryPos1;
    coordinate currentVector = currentPos - trajectoryPos1;
    double currentToPos1 = distanceBetweenPoints(currentPos, trajectoryPos1);
    double dotProduct = trajectoryVector.x * currentVector.x + trajectoryVector.z * currentVector.z;
    return sqrt(pow(currentToPos1, 2) - pow(dotProduct, 2));
}

bool obstacle_in_robot_path(coordinate robot_pos, coordinate obstacle_pos, coordinate path_vector, double target_distance, double clearance) {
  coordinate perp_path_bearing_vector(-path_vector.z, path_vector.x);

  coordinate displacement_from_robot = obstacle_pos - robot_pos;
  double distance_parallel_to_path = displacement_from_robot.x * path_vector.x + displacement_from_robot.z * path_vector.z;
  if (distance_parallel_to_path > 0 && distance_parallel_to_path < target_distance) {
    //only consider blocks infront of the robots path
    // using cross product - positive is in clockwise rotation from path
    double distance_perp_to_path = displacement_from_robot.z * path_vector.x - displacement_from_robot.x * path_vector.z;
    if (abs(distance_perp_to_path) < clearance) {
      cout << "Collision with obstacle at " << obstacle_pos << endl;
      return true;
    }
  }
  return false;
}