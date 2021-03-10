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

const PIDGains RotationalPIDGains{ 0.1, 0, 0, 0.1, 10};
const double endTurnThresh = 0.1;  // raise reached rotation flag when less than 0.1 degrees from the target rotation
const double turnOnlyThresh = 2;  // if more than this many degrees from the correct heading dont move forwards
const double noTurnThresh = 0.025;  // (m^2) If less than the root of this distance away from the target dont aapply rotation corrections any more
const double endMoveThresh = 0.00025; // (m^2) If less than the root of this distance away from the target dont move any more
const double distanceMeasurementWeight = 0.5;  // how much weight to give new distance sensor measurements when recalculating block position

PIDState RotationalPIDState{ 0, 0 };

const PIDGains ForwardPIDGains{ 1000, 0, 0, 0, 0.01};
PIDState ForwardPIDState{ 0, 0 };

coordinate targetPosition;
bool turningStage = false;
bool forwardStage = false;
bool reachedPosition = false;

double targetBearing; 
bool reachedBearing = false;

void updateTargetBearing(double newBearing) {
  targetBearing = newBearing;
  reachedBearing = false;
}

void updateTargetPosition(coordinate newTarget) {
  targetPosition = newTarget;
  turningStage = true;
  forwardStage = false;
  reachedPosition = false;

  ForwardPIDState = PIDState{ 0, 0 };
  RotationalPIDState = PIDState{ 0, 0 };
}

void tweakBlockDistanceFromMeasurement(coordinate robotPosition, const double* currentBearingVector, double distance) {
  coordinate rotatedSensorDisp = rotateVector(distanceSensorDisplacement, getCompassBearing(currentBearingVector));
  coordinate displacementFromDistanceSensor = targetPosition - robotPosition - rotatedSensorDisp;

  double expectedDist = displacementFromDistanceSensor.x * -currentBearingVector[0] + displacementFromDistanceSensor.z * currentBearingVector[2];
  double averagedDist = expectedDist * (1 - distanceMeasurementWeight) + distance * distanceMeasurementWeight;

  if (expectedDist > ULTRASOUND_MIN_DISTANCE) {
    targetPosition.x += (distance - frontOfRobotDisplacement.x - expectedDist) * -currentBearingVector[0] * distanceMeasurementWeight;
    targetPosition.z += (distance - frontOfRobotDisplacement.z - expectedDist) * currentBearingVector[2] * distanceMeasurementWeight;
  }
}

bool hasReachedPosition() {
  return reachedPosition;
}

bool hasFinishedTurning() {
  return !turningStage;
}

bool hasReachedTargetBearing() {
  return reachedBearing;
}

tuple<double, double> turnToTargetBearing(const double* currentBearingVector) {
  double current_bearing = getCompassBearing(currentBearingVector);
  double turning_speed = turnToBearing(targetBearing, current_bearing);
  if (turning_speed == 0) {
    reachedBearing = true;
  }
  return tuple<double, double>(turning_speed, -turning_speed);
}

tuple<double, double> moveToPosition(coordinate currentPosition, const double* currentBearingVector) {

  coordinate displacement = targetPosition - currentPosition;
  double target_bearing = getBearing(displacement);
  double current_bearing = getCompassBearing(currentBearingVector);

  double turning_speed = 0.0;
  double forward_speed = 0.0;

  double distance_to_target = displacement.x * -currentBearingVector[0]
    + displacement.z * currentBearingVector[2];  // dot product taking into account the compass orientation

  if (turningStage) {
    turning_speed = turnToBearing(target_bearing, current_bearing);
    if (abs(getBearingDifference(current_bearing, target_bearing)) < turnOnlyThresh) {
      forwardStage = true;
    }
  }
  if (forwardStage) {
    forward_speed = clamp(getPIDOutput(distance_to_target, ForwardPIDGains, ForwardPIDState), -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
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

coordinate getPositionInfrontOfBlock(coordinate blockPosition, coordinate robotPosition) {
  coordinate displacement = blockPosition - robotPosition;
  double target_bearing = getBearing(displacement);

  coordinate rotatedGrabberDisp = rotateVector(frontOfRobotDisplacement, target_bearing);
  return blockPosition - rotatedGrabberDisp;
}

coordinate getBlockPosition(tuple<double, double> afterLastJump, tuple<double, double> beforeJump, bool lastJumpWasFall, bool jumpWasFall,
  coordinate robotPosition, const double sensorBeamAngle) {
  double blockAvgDistance = 0;
  double blockAvgAngle = 0;

  if (abs(getBearingDifference(get<1>(afterLastJump), get<1>(beforeJump))) >= sensorBeamAngle) {
    // block is not being obscured by anything else
    cout << "no obstruction" << endl;
    blockAvgDistance = (get<0>(afterLastJump) + get<0>(beforeJump)) / 2;
    blockAvgAngle = (get<1>(afterLastJump) + get<1>(beforeJump)) / 2;
  }
  else if (lastJumpWasFall && jumpWasFall) {
    cout << "obstruction to right" << endl;
    // block partially obscured by something infront of it at the new jump side
    blockAvgDistance = (get<0>(afterLastJump) + get<0>(beforeJump)) / 2;
    blockAvgAngle = get<1>(afterLastJump) + (sensorBeamAngle + BLOCK_SIZE * RAD_TO_DEG / blockAvgDistance) / 2;
  }
  else if (!lastJumpWasFall && !jumpWasFall) {
    cout << "obstruction to left" << endl;
    blockAvgDistance = (get<0>(afterLastJump) + get<0>(beforeJump)) / 2;
    blockAvgAngle = get<1>(beforeJump) - (sensorBeamAngle + BLOCK_SIZE * RAD_TO_DEG / blockAvgDistance) / 2;
  }
  else {
    // in this case we still have some information on the block, but choose to 
    // ignore it because it cannot be accurately located
    cout << "Not possible to accurately find block, ignoring" << endl;
  }

  coordinate rotatedSensorDisp = rotateVector(distanceSensorDisplacement, blockAvgAngle);
  double block_x = robotPosition.x + rotatedSensorDisp.x + blockAvgDistance * cos(blockAvgAngle * DEG_TO_RAD);
  double block_z = robotPosition.z + rotatedSensorDisp.z + blockAvgDistance * sin(blockAvgAngle * DEG_TO_RAD);
  return coordinate(block_x, block_z);
}