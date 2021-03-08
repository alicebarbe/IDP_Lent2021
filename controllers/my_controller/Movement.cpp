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

using namespace std;
using namespace webots;

const PIDGains RotationalPIDGains{ 0.1, 0, 0, 0.1, 10};
const double endTurnThresh = 0.1;  // raise reached rotation flag when less than 0.1 degrees from the target rotation
const double turnOnlyThresh = 2;  // if more than this many degrees from the correct heading dont move forwards
const double noTurnThresh = 0.025;  // (m^2) If less than the root of this distance away from the target dont aapply rotation corrections any more
const double endMoveThresh = 0.00025; // (m^2) If less than the root of this distance away from the target dont move any more
const double distanceMeasurementWeight = 0.2;  // how much weight to give new distance sensor measurements when recalculating block position

PIDState RotationalPIDState{ 0, 0 };

const PIDGains ForwardPIDGains{ 1000, 0, 0, 0, 0.01};
PIDState ForwardPIDState{ 0, 0 };

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

void tweakBlockDistanceFromMeasurement(tuple<double, double> robotPosition, const double* currentBearingVector, double distance) {
  tuple<double, double> rotatedSensorDisp = rotateVector(distanceSensorDisplacement, getCompassBearing(currentBearingVector));
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

tuple<double, double> moveToPosition(tuple<double, double> currentPosition, const double* currentBearingVector) {

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

tuple<double, double> getPositionInfrontOfBlock(tuple<double, double> blockPosition, tuple<double, double> robotPosition) {
  tuple<double, double> displacement(get<0>(blockPosition) - get<0>(robotPosition), get<1>(blockPosition) - get<1>(robotPosition));
  double target_bearing = getBearing(displacement);

  tuple<double, double> rotatedGrabberDisp = rotateVector(frontOfRobotDisplacement, target_bearing);
  return tuple<double, double>(get<0>(blockPosition) - get<0>(rotatedGrabberDisp), get<1>(blockPosition) - get<1>(rotatedGrabberDisp));
}

tuple<double, double> getBlockPosition(tuple<double, double> afterLastJump, tuple<double, double> beforeJump, bool lastJumpWasFall, bool jumpWasFall,
  const double* robot_pos, const double sensorBeamAngle) {
  double blockAvgDistance = 0;
  double blockAvgAngle = 0;

  if (abs(getBearingDifference(get<1>(afterLastJump), get<1>(beforeJump))) >= sensorBeamAngle) {
    // block is not being obscured by anything else
    blockAvgDistance = (get<0>(afterLastJump) + get<0>(beforeJump)) / 2;
    blockAvgAngle = (get<1>(afterLastJump) + get<1>(beforeJump)) / 2;
  }
  else if (lastJumpWasFall && jumpWasFall) {
    // block partially obscured by something infront of it at the new jump side
    blockAvgDistance = (get<0>(afterLastJump) + get<0>(beforeJump)) / 2;
    blockAvgAngle = get<1>(afterLastJump) + (sensorBeamAngle + BLOCK_SIZE * RAD_TO_DEG / blockAvgDistance) / 2;
  }
  else if (!lastJumpWasFall && !jumpWasFall) {
    blockAvgDistance = (get<0>(afterLastJump) + get<0>(beforeJump)) / 2;
    blockAvgAngle = get<1>(beforeJump) - (sensorBeamAngle + BLOCK_SIZE * RAD_TO_DEG / blockAvgDistance) / 2;
  }
  else {
    // in this case we still have some information on the block, but choose to 
    // ignore it because it cannot be accurately located
    cout << "Not possible to accurately find block, ignoring" << endl;
  }

  tuple<double, double> rotatedSensorDisp = rotateVector(distanceSensorDisplacement, blockAvgAngle);
  double block_x = robot_pos[0] + get<0>(rotatedSensorDisp) + blockAvgDistance * cos(blockAvgAngle * DEG_TO_RAD);
  double block_z = robot_pos[2] + get<1>(rotatedSensorDisp) + blockAvgDistance * sin(blockAvgAngle * DEG_TO_RAD);
  return tuple<double, double>(block_x, block_z);
}