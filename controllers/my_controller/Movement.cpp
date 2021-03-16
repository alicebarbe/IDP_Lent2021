#define _USE_MATH_DEFINES
#include <tuple>
#include <algorithm>
#include <math.h>
#include <cstdlib>
#include <cmath>

#include "CommunicationClient.hpp"
#include "Movement.hpp"
#include "SimulationParameters.hpp"
#include "Utility.hpp"
#include "Coordinate.hpp"

using namespace std;
using namespace webots;

const PIDGains RotationalPIDGains{ 5, 0, 0.01, 0.5, 10};
const double endTurnThresh = 0.1;  // raise reached rotation flag when less than 0.1 degrees from the target rotation
const double turnOnlyThresh = 2;  // if more than this many degrees from the correct heading dont move forwards
const double noTurnThresh = 0.025;  // (m^2) If less than the root of this distance away from the target dont aapply rotation corrections any more
const double endMoveThresh = 0.00025; // (m^2) If less than the root of this distance away from the target dont move any more
const double distanceMeasurementWeight = 0.5;  // how much weight to give new distance sensor measurements when recalculating block position

PIDState RotationalPIDState{ 0, 0 };

const PIDGains ForwardPIDGains{ 1000, 0.1, 0, 0, 0.05};
PIDState ForwardPIDState{ 0, 0 };

coordinate targetPosition;
bool turningStage = false;
bool forwardStage = false;
bool reachedPosition = false;
bool useDistanceSensor = true;
bool canReverse = false;
bool maintainingBearing = false;

double targetBearing; 
bool reachedBearing = false;

void updateTargetBearing(double newBearing) {
  targetBearing = newBearing;
  reachedBearing = false;
}

void updateTargetPosition(coordinate newTarget, bool reverse) {
  targetPosition = get<1>(offsetPointAwayFromWall(newTarget, closestDistanceBlockFromWall, closestDistanceBlockFromWall));
  turningStage = true;
  forwardStage = false;
  reachedPosition = false;
  useDistanceSensor = true;
  maintainingBearing = false;
  canReverse = reverse;

  ForwardPIDState = PIDState{ 0, 0 };
  RotationalPIDState = PIDState{ 0, 0 };
}

void updateTargetDistance(coordinate newTarget, bool reverse) {
  targetPosition = get<1>(offsetPointAwayFromWall(newTarget, closestDistanceBlockFromWall, closestDistanceBlockFromWall));
  turningStage = false; // disable rotation, so only move forwards
  forwardStage = true;
  reachedPosition = false;
  useDistanceSensor = true;
  maintainingBearing = false;
  canReverse = reverse;

  ForwardPIDState = PIDState{ 0, 0 };
  RotationalPIDState = PIDState{ 0, 0 };
}

bool tweakTargetDistanceFromMeasurement(coordinate robotPosition, const double* currentBearingVector, double distance, double lostThreshold) {
  if (distance < ULTRASOUND_MIN_DISTANCE) {
    useDistanceSensor = false;
  }
  coordinate rotatedSensorDisp = rotateVector(distanceSensorDisplacement, getCompassBearing(currentBearingVector));
  coordinate displacementFromDistanceSensor = targetPosition - robotPosition - rotatedSensorDisp;

  double expectedDist = getExpectedDistanceOfBlock(robotPosition, currentBearingVector);
  if ((abs(distance - expectedDist) > lostThreshold) || (abs(robotPosition.x) > 1.03 && distance > 0.04) || (abs(robotPosition.z) > 1.03 && distance > 0.04)) {
    // provision if block is so far from where it should be that the block is gone
    return false;
  }
  double averagedDist = expectedDist * (1 - distanceMeasurementWeight) + distance * distanceMeasurementWeight;

  targetPosition.x += (distance - frontOfRobotDisplacement.x - expectedDist) * -currentBearingVector[0] * distanceMeasurementWeight;
  if (targetPosition.x > 1.03) targetPosition.x = 1.03;
  if (targetPosition.x < -1.03) targetPosition.x = -1.03;
  targetPosition.z += (distance - frontOfRobotDisplacement.z - expectedDist) * currentBearingVector[2] * distanceMeasurementWeight;
  if (targetPosition.z > 1.03) targetPosition.z = 1.03;
  if (targetPosition.z < -1.03) targetPosition.z = -1.03;
  return true;
}

bool hasReachedPosition() {
  return reachedPosition;
}

bool canUseDistanceSensor() {
  return !turningStage && useDistanceSensor;
}

bool isMaintainingTargetBearing() {
  return maintainingBearing;
}

bool hasReachedTargetBearing() {
  return reachedBearing;
}

tuple<double, double> updateRotationControlLoop(const double* currentBearingVector) {
  double current_bearing = getCompassBearing(currentBearingVector);
  double turning_speed = getBearingCorrection(targetBearing, current_bearing);
  if (turning_speed == 0) {
    reachedBearing = true;
  }
  return tuple<double, double>(turning_speed, -turning_speed);
}

tuple<double, double> updatePositionalControlLoop(coordinate currentPosition, const double* currentBearingVector) {

  coordinate displacement = targetPosition - currentPosition;

  double turning_speed = 0.0;
  double forward_speed = 0.0;

  double distance_to_target = displacement.x * -currentBearingVector[0]
    + displacement.z * currentBearingVector[2];  // dot product taking into account the compass orientation

  if (turningStage) {
    double target_bearing = getBearing(displacement);
    double current_bearing = getCompassBearing(currentBearingVector);

    turning_speed = getBearingCorrection(target_bearing, current_bearing);
    maintainingBearing = false;
    if (abs(getBearingDifference(current_bearing, target_bearing)) < turnOnlyThresh) {
      forwardStage = true;
      maintainingBearing = true;
    }
  }
  if (forwardStage) {
    forward_speed = clamp(getPIDOutput(distance_to_target, ForwardPIDGains, ForwardPIDState), -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
    if (turningStage && abs(distance_to_target) < noTurnThresh) {
      turningStage = false;
    }
    if (forwardStage && abs(distance_to_target) < endMoveThresh) {
      forwardStage = false;
      reachedPosition = true;
    }
  }

  double left_motor_speed = clamp(forward_speed + turning_speed, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
  double right_motor_speed = clamp(forward_speed - turning_speed, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
  return tuple<double, double>(left_motor_speed, right_motor_speed);
}

double getBearingCorrection(double bearing, double currentBearing) {
  double error = getBearingDifference(currentBearing, bearing);
  double turn_speed = clamp(getPIDOutput(error, RotationalPIDGains, RotationalPIDState), - MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
  return turn_speed;
}

coordinate getPositionAroundBlock(coordinate blockPosition, coordinate robotPosition, coordinate offset) {
  coordinate displacement = blockPosition - robotPosition;
  double target_bearing = getBearing(displacement);

  coordinate rotatedOffset = rotateVector(offset, target_bearing);
  return blockPosition - rotatedOffset;
}

coordinate getBlockPosition(tuple<double, double> afterLastJump, tuple<double, double> beforeJump, bool lastJumpWasFall, bool jumpWasFall,
  coordinate robotPosition, const double sensorBeamAngle) {
  double blockAvgAngle = 0;
  double blockAvgDistance = (get<0>(afterLastJump) + get<0>(beforeJump)) / 2;

  //cout << "block width: " << abs(getBearingDifference(get<1>(afterLastJump), get<1>(beforeJump))) << "Expected block width: " << BLOCK_SIZE * RAD_TO_DEG / blockAvgDistance << endl;
  if (abs(getBearingDifference(get<1>(afterLastJump), get<1>(beforeJump))) >= sensorBeamAngle + 0.6 * BLOCK_SIZE * RAD_TO_DEG / blockAvgDistance) {
    // block is not being obscured by anything else
    blockAvgAngle = (get<1>(afterLastJump) + get<1>(beforeJump)) / 2;
  }
  else if (lastJumpWasFall && jumpWasFall) {
    // block partially obscured by something infront of it at the new jump side
    blockAvgAngle = get<1>(afterLastJump) + (sensorBeamAngle + BLOCK_SIZE * RAD_TO_DEG / blockAvgDistance) / 2;
  }
  else if (!lastJumpWasFall && !jumpWasFall) {
    blockAvgAngle = get<1>(beforeJump) - (sensorBeamAngle + BLOCK_SIZE * RAD_TO_DEG / blockAvgDistance) / 2;
  }
  else {
    // in this case we still have some information on the block, but choose to 
    // ignore it because it cannot be accurately located
    cout << "Not possible to accurately find block, ignoring" << endl;
  }

  return getBlockPositionFromAngleAndDistance(robotPosition, blockAvgDistance, blockAvgAngle);
}

coordinate getBlockPositionFromAngleAndDistance(coordinate robotPosition, double blockAvgDistance, double blockAvgAngle) {
  coordinate rotatedSensorDisp = rotateVector(distanceSensorDisplacement, blockAvgAngle);
  double block_x = robotPosition.x + rotatedSensorDisp.x + blockAvgDistance * cos(blockAvgAngle * DEG_TO_RAD);
  double block_z = robotPosition.z + rotatedSensorDisp.z + blockAvgDistance * sin(blockAvgAngle * DEG_TO_RAD);
  return coordinate(block_x, block_z);
}

double getWallDistance(const coordinate robotPos, double angle, coordinate sensorDisp) {
  double boundXPos, boundXNeg, boundZPos, boundZNeg;
  double radAngle = angle * DEG_TO_RAD;
  coordinate rotatedSensorDisp = rotateVector(sensorDisp, angle);

  boundXPos = (ARENA_X_MAX - robotPos.x - rotatedSensorDisp.x) / cos(radAngle);
  boundXNeg = (ARENA_X_MIN - robotPos.x - rotatedSensorDisp.x) / cos(radAngle);

  boundZPos = (ARENA_Z_MAX - robotPos.z - rotatedSensorDisp.z) / sin(radAngle);
  boundZNeg = (ARENA_Z_MIN - robotPos.z - rotatedSensorDisp.z) / sin(radAngle);

  return min(max(boundXPos, boundXNeg), max(boundZNeg, boundZPos));
}

double getWallCollisionDistance(const coordinate robotPos, double angle) {
  // note we have to generate a coordinate from the const coordinate in order to add/subtract
  return min(getWallDistance(robotPos, angle, coordinate(distanceSensorDisplacement) + rightMostPointDispacement), 
    getWallDistance(robotPos, angle, coordinate(distanceSensorDisplacement) - rightMostPointDispacement));
}

coordinate getBlockPositionInGrabber(coordinate robotPosition, double bearing) {
  coordinate rotatedFrontDisp = rotateVector(frontOfRobotDisplacement, bearing);
  return robotPosition + rotatedFrontDisp;
}

double getExpectedDistanceOfBlock(coordinate robotPosition, const double* currentBearingVector) {
  coordinate rotatedSensorDisp = rotateVector(distanceSensorDisplacement, getCompassBearing(currentBearingVector));
  coordinate displacementFromDistanceSensor = targetPosition - robotPosition - rotatedSensorDisp;

  return displacementFromDistanceSensor.x * -currentBearingVector[0] + displacementFromDistanceSensor.z * currentBearingVector[2];
}

tuple<bool, coordinate> offsetPointAwayFromWall(coordinate blockPos, double distanceFromWallThresh, double targetOffset) {
  coordinate offsetPoint = blockPos;
  bool isViaPoint = false;

  if (ARENA_X_MAX - blockPos.x < distanceFromWallThresh) {
    // North wall
    offsetPoint.x = ARENA_X_MAX - targetOffset;
    isViaPoint = true;
  }
  else if (blockPos.x - ARENA_X_MIN < distanceFromWallThresh) {
    // South wall
    offsetPoint.x = ARENA_X_MIN + targetOffset;
    isViaPoint = true;
  }
  if (blockPos.z - ARENA_Z_MIN < distanceFromWallThresh) {
    // West wall
    offsetPoint.z = ARENA_Z_MIN + targetOffset;
    isViaPoint = true;
  }
  else if (ARENA_Z_MAX - blockPos.z < distanceFromWallThresh) {
    // East wall
    offsetPoint.z = ARENA_Z_MAX - targetOffset;
    isViaPoint = true;
  }
  return tuple<bool, coordinate>(isViaPoint, offsetPoint);
}

coordinate getTargetPosition() {
  return targetPosition;
}