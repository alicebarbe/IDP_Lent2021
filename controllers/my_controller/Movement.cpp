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

const PIDGains RotationalPIDGains{ 0.1, 0, 0, 0.1, 10};
const double endTurnThresh = 0.1;  // raise reached rotation flag when less than 0.1 degrees from the target rotation
const double turnOnlyThresh = 2;  // if more than this many degrees from the correct heading dont move forwards
const double noTurnThresh = 0.025;  // (m^2) If less than the root of this distance away from the target dont aapply rotation corrections any more
const double endMoveThresh = 0.00025; // (m^2) If less than the root of this distance away from the target dont move any more

PIDState RotationalPIDState{ 0, 0 };

const PIDGains ForwardPIDGains{ 1000, 0, 0, 0, 0.01};
PIDState ForwardPIDState{ 0, 0 };

const double distanceMeasurementWeight = 0.2;  // how much weight to give new distance sensor measurements when recalculating block position

tuple<double, double> targetPosition;
bool turningStage = false;
bool forwardStage = false;
bool reachedPosition = false;

void updateTargetPosition(tuple<double, double> newTarget) {
  targetPosition = newTarget;
  turningStage = true;
  forwardStage = false;
  reachedPosition = false;

  ForwardPIDState = PIDState{ 0, 0 };
  RotationalPIDState = PIDState{ 0, 0 };
}

void tweakBlockDistanceFromMeasurement(tuple<double, double> robotPosition, tuple<double, double> sensorDisplacement, tuple<double, double> frontOfRobotDisplacement, const double* currentBearingVector, double distance) {
  tuple<double, double> rotatedSensorDisp = rotateVector(sensorDisplacement, getCompassBearing(currentBearingVector));
  tuple<double, double> displacementFromDistanceSensor = tuple<double, double>(get<0>(targetPosition) - get<0>(robotPosition) - get<0>(rotatedSensorDisp),
    get<1>(targetPosition) - get<1>(robotPosition) - get<1>(rotatedSensorDisp));
  double expectedDist = get<0>(displacementFromDistanceSensor) * currentBearingVector[2] + get<0>(displacementFromDistanceSensor) * currentBearingVector[0];
  double averagedDist = expectedDist * (1 - distanceMeasurementWeight) + distance * distanceMeasurementWeight;

  cout << "Expected distance: " << expectedDist << "   actual distance:" << distance << endl;
  get<0>(targetPosition) += (distance - get<0>(frontOfRobotDisplacement) - expectedDist) * currentBearingVector[2] * distanceMeasurementWeight;
  get<1>(targetPosition) += (distance - get<0>(frontOfRobotDisplacement) - expectedDist) * currentBearingVector[0] * distanceMeasurementWeight;
}

bool hasReachedPosition() {
  return reachedPosition;
}

bool hasFinishedTurning() {
  return !turningStage;
}

tuple<double, double> moveToPosition(tuple<double, double> currentPosition,
                                     const double* currentBearingVector) {

  tuple<double, double> displacement(get<0>(targetPosition) - get<0>(currentPosition), get<1>(targetPosition) - get<1>(currentPosition));
  double target_bearing = getBearing(displacement);
  double current_bearing = getCompassBearing(currentBearingVector);

  double turning_speed = 0.0;
  double forward_speed = 0.0;

  double distance_to_target = get<0>(displacement) * currentBearingVector[2]
    + get<1>(displacement) * currentBearingVector[0];  // dot product taking into account the compass orientation

  if (turningStage) {
    turning_speed = turnToBearing(target_bearing, current_bearing);
    if (abs(getBearingDifference(current_bearing, target_bearing)) < turnOnlyThresh) {
      forwardStage = true;
    }
  }
  if (forwardStage) {
    forward_speed = clamp(getPIDOutput(distance_to_target, ForwardPIDGains, ForwardPIDState), -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
    // cout << distance_to_target << endl;
    if (abs(distance_to_target) < noTurnThresh) {
      turningStage = false;
    }
    if (abs(distance_to_target) < endMoveThresh) {
      forwardStage = false;
      reachedPosition = true;
    }
  }

  double left_motor_speed = clamp(forward_speed + turning_speed, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
  double right_motor_speed = clamp(forward_speed - turning_speed, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
  return tuple<double, double>(left_motor_speed, right_motor_speed);
}

double turnToBearing(double bearing, double currentBearing) {
  double error = getBearingDifference(currentBearing, bearing);
  double turn_speed = clamp(getPIDOutput(error, RotationalPIDGains, RotationalPIDState), - MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
  return turn_speed;
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

tuple<double, double> rotateVector(const tuple<double, double> vector, double angle) {
  double radAngle = angle * M_PI / 180.0;
  double rotatedX = get<0>(vector) * cos(radAngle) - get<1>(vector) * sin(radAngle);
  double rotatedZ = get<0>(vector) * sin(radAngle) + get<1>(vector) * cos(radAngle);
  return tuple<double, double>(rotatedX, rotatedZ);
}