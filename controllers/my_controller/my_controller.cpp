
// File:          my_controller.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdlib.h>
#include <iostream>
#include <tuple>
#include <vector>
#include <numeric>
#include <cmath>
#include <string>

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>

#include "CommunicationClient.hpp"
#include "SimulationParameters.hpp"
#include "Sensors.hpp"
#include "Movement.hpp"
#include "Motors.hpp"
#include "Servos.hpp"
#include "Utility.hpp"
#include "Coordinate.hpp"

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;

int robotColour;
string robotColourStr;
#define coutWithName cout << "I am " <<  robotColourStr << " and "

bool emergencyChecker(void* emergencyParams);
bool bypassEmergencyChecker(void* emergencyParams);
vector<coordinate> scanForBlocks(bool scanning_whole_arena, bool (*emergencyFunc)(void*), void* emergencyParams = NULL);
void dealwithblock(bool (*emergencyFunc)(void*), void* emergencyParams = NULL);
void moveForward(double distance, bool positionIsBlock, bool (*emergencyFunc)(void*), void* emergencyParams = NULL);
bool moveToPosition(coordinate blockPosition, bool positionIsBlock, bool (*emergencyFunc)(void*), void* emergencyParams = NULL, bool canReverse = false);
void turnToBearing(double bearing, bool (*emergencyFunc)(void*), void* emergencyParams = NULL);
void timeDelay(int delayLength, bool (*emergencyFunc)(void*), void* emergencyParams = NULL);
void waitUntilCollisionEventFinished();
void sendBlockPositions(vector<coordinate> blockPositions);
bool confirmBlockPosition();
bool relocateBlock(coordinate& nextTarget, bool (*emergencyFunc)(void*), void* emergencyParams = NULL);
void handleBlockLost();

int blocks_collected = 0;
bool isMovingToPosition = false;
bool isAvoidingCollision = false;

// Webots sensors/actuators
Robot *robot = new Robot();
tuple<Motor*, Motor*> motors = initMotors(robot, "wheel1", "wheel2");
Motor* gripperservo = initServo(robot, "gripper motor");
Motor* trapdoorservo = initServo(robot, "trap_door");
DistanceSensor* ds1 = initDistanceSensor(robot, "ds");
GPS* gps = initGPS(robot, "gps");
Compass* compass = initCompass(robot, "compass");
tuple<LightSensor*, LightSensor*> colourSensor = initLightSensor(robot, "light_sensor_red", "light_sensor_green");
Receiver* receiver = initReceiver(robot, "receiver");
Emitter* emitter = initEmitter(robot, "emitter");

int timeStep = (int)robot->getBasicTimeStep();
coordinate otherRobotPosition;
int emergencyCounterMax = 50; // get position every this many iterations

coordinate currentDestination; // where we're headed
coordinate otherRobotDestination; // where the other robot is headed

int main(int argc, char** argv) {
  // get the time step of the current world.
    timeDelay(5, emergencyChecker);

  // identify what colour robot 

  if (getLocation(gps).z > 0) robotColour = RED_ROBOT; else robotColour = GREEN_ROBOT;
  sayHello(robotColour, emitter);
  sendRobotLocation(gps, robotColour, emitter);

  if (robotColour == 1) {
      robotColourStr = "GREEN";
  }
  else if (robotColour) {
      robotColourStr = "RED";
  }

  while (robot->step(timeStep) != -1) {
    //coutWithName << "In main" << endl;
    message* receivedData = receiveData(receiver);
    if (receivedData) {
      vector<coordinate> targetPoints;
      coordinate blockPosition;

      if (floor(get<0>(*receivedData) / 100) == robotColour) {
        cout << "Robot " << robotColour << " : " << get<0>(*receivedData) << " , " << get<1>(*receivedData) << ", " << get<2>(*receivedData) << endl;
        switch (get<0>(*receivedData) % 100) {
        case(80):
            if (robotColour == 1) {
                moveToPosition(coordinate(0, -0.4), false, emergencyChecker);
            }
            else if (robotColour == 2) {
                moveToPosition(coordinate(0, 0.4), false, emergencyChecker);
          }
          turnToBearing((robotColour == RED_ROBOT) ? 60 : 240, emergencyChecker);
          targetPoints = scanForBlocks(0, emergencyChecker);
          sendRobotLocation(gps, robotColour, emitter);
          sendBlockPositions(targetPoints);
          sendFinishedScan(robotColour, emitter);
          break;
        case(00): {
          blockPosition = coordinate(get<1>(*receivedData), get<2>(*receivedData));
          currentDestination = blockPosition;
          if (moveToPosition(blockPosition, true, emergencyChecker)) {
            dealwithblock(emergencyChecker);
          }
          else {
            sendDealtwithBlock(robotColour, emitter);
          }
          break;
        }
        case(90): {
            currentDestination = coordinate(get<1>(*receivedData), get<2>(*receivedData));
            coutWithName << "my destination is : " << currentDestination << endl;
            //TO-DO: change the below to currentDestination instead?
            moveToPosition(currentDestination, false, emergencyChecker, NULL, false);
            // moveForward(frontOfRobotDisplacement.x, bypassEmergencyChecker);
            sendDealtwithBlock(robotColour, emitter);
            closeTrapDoor(trapdoorservo);    //when we go home for the final time we need to shut the trapdoor so we can scan again if needed
            break;
        }
        case(99):
          coutWithName << "something bad has happened";
          break;
        case(98):
          coutWithName << "Stopping for other robot" << endl;
          setMotorVelocity(motors, tuple<double, double>(0.0, 0.0));
          waitUntilCollisionEventFinished();
          sendDealtwithBlock(robotColour, emitter);
          break;
        case(96):
          coutWithName << "Ending collision event" << endl;
          isAvoidingCollision = false;
        }
      }
      
      if (floor(get<0>(*receivedData) / 100) == (3 - robotColour)) {
          // cout << "Robot " << robotColour << " From other robot : " << get<0>(*receivedData) << " , " << get<1>(*receivedData) << ", " << get<2>(*receivedData) << endl;
          if (get<0>(*receivedData) % 100 == 20) {
              otherRobotPosition = coordinate(get<1>(*receivedData), get<2>(*receivedData));
              cout << otherRobotPosition << endl;
          }
      }
    }
  }
  delete robot;
  return 0;
}

bool bypassEmergencyChecker(void* emergencyParams) {
  // use this to avoid checking emergencies within an emergency
  return false;
}

bool emergencyChecker(void* emergencyParams) {
  //Put any emergency checking here
  // This function can be blocking (ie with a loop in it)
  // return false to resume the last action, 
  // return true to cancel out of the last action - 
  // Note that some blocking actions call others therefore 
  // exiting out does not always take you to the very top - this should likely be change
  static int emergencyCounter = 0; // to avoid polling position every single time
  message* receivedData = receiveData(receiver);
  if (receivedData) {
    if (floor(get<0>(*receivedData) / 100) == robotColour) {
      switch (get<0>(*receivedData) % 100) {
      case(99):
        coutWithName << "something bad has happened" << endl;
        timeDelay(5, bypassEmergencyChecker);
        return false;
        break;
      case(98):
        coutWithName << "stopping for other robot" << endl;
        setMotorVelocity(motors, tuple<double, double>(0.0, 0.0));
        waitUntilCollisionEventFinished();
        return false;
        break;
      case(96):
        coutWithName << "ending collision event" << endl;
        isAvoidingCollision = false;
      }
    }
    if (floor(get<0>(*receivedData) / 100) == (3 - robotColour)) {
      if (get<0>(*receivedData) % 100 == 20) {
        //coutWithName << "Updating other robot position" << endl;
        otherRobotPosition = coordinate(get<1>(*receivedData), get<2>(*receivedData));
      }
      if (get<0>(*receivedData) % 100 == 25) {
        otherRobotDestination = coordinate(get<1>(*receivedData), get<2>(*receivedData));
      }
    }
  }

  // make green robot stop if the red robot is too close
  if (emergencyCounter >= emergencyCounterMax) {
    //coutWithName << "Running emergency checker" << endl;
    emergencyCounter = 0;
    coordinate currentRobotPosition = getLocation(gps);

    if (isMovingToPosition && !isAvoidingCollision) {
      const double* compassVector = getDirection(compass);
      coordinate bearingVector(-compassVector[0], compassVector[2]);
      double target_distance = (getTargetPosition() - currentRobotPosition).x * bearingVector.x + (getTargetPosition() - currentRobotPosition).z * bearingVector.z + forwardMostRobotPointDist;
      if (obstacle_in_robot_path(currentRobotPosition, otherRobotPosition, bearingVector, min(target_distance, otherRobotProximityThresh), robotHalfWidthClearance + collisionCircleRadius)) {
        coutWithName << "other robot incoming! " << endl;
        sendRobotCollisionMessage(robotColour, emitter);
        isAvoidingCollision = true;
        return true;
      }
    }
  }
  emergencyCounter++;

  return false;
}

void sendBlockPositions(vector<coordinate> blockPositions) {
  for (auto it = blockPositions.begin(); it != blockPositions.end(); it++) {
    sendBlockLocation(*it, robotColour, emitter);
  }
}

vector<coordinate> scanForBlocks(bool scanning_whole_arena, bool (*emergencyFunc)(void*), void* emergencyParams) {
  const int measureTimeInterval = 1;  // wait this many simulation timesteps before measuring
  const tuple<double, double> motorTurnSpeed(0.5, -0.5);
  const double blockDetectThresh = 0.05;  // detect changes in ultrasound measuruments greater than this
  const double wallSeparationThresh = 0.08;   // detect blocks if they are this far from the wall
  double angleToRotate = 0;
  if (!scanning_whole_arena) angleToRotate = 200;
  else angleToRotate = 360;

  int i = 0;

  vector<coordinate> blockPositions;

  double lastDistance = getDistanceMeasurement(ds1);
  double distance = lastDistance;
  double lastBearing = getCompassBearing(getDirection(compass));
  double bearing = lastBearing;
  double wallDistance;
  coordinate robotPosition;

  bool firstJump = true;
  tuple<double, double> afterLastJump;
  bool lastJumpWasFall = true;
  tuple<double, double> beforeJump, afterJump;
  bool jumpWasFall = true;

  double startBearing = bearing;
  double endBearing = constrainBearing(startBearing + angleToRotate);
  bool crossedNorthOnce = false;
  setMotorVelocity(motors, motorTurnSpeed);

  openGripper(gripperservo);
  closeTrapDoor(trapdoorservo); timeDelay(15, emergencyFunc, emergencyParams);

  while (robot->step(timeStep) != -1) {
    lastDistance = distance;
    lastBearing = bearing;
    distance = getDistanceMeasurement(ds1);
    bearing = getCompassBearing(getDirection(compass));

    if (lastBearing < endBearing && bearing >= endBearing) {
      setMotorVelocity(motors, tuple<double, double>(0, 0));
      break;
    }

    i = (i + 1) % measureTimeInterval;

    if (i == 0) {
      robotPosition = getLocation(gps);
      wallDistance = getWallDistance(robotPosition, bearing);

      if (abs(distance - lastDistance) > blockDetectThresh) {
        afterLastJump = afterJump;
        lastJumpWasFall = jumpWasFall;

        beforeJump = make_tuple(lastDistance, lastBearing);
        afterJump = make_tuple(distance, bearing);
        jumpWasFall = (distance - lastDistance < 0);

        if (firstJump) {
          // handle the first jump case
          afterLastJump = afterJump;
          lastJumpWasFall = jumpWasFall;

          firstJump = false;
          continue;
        }

        if (get<0>(beforeJump) < wallDistance - wallSeparationThresh) {
          blockPositions.push_back(getBlockPosition(afterLastJump, beforeJump, lastJumpWasFall, jumpWasFall, robotPosition, ULTRASOUND_BEAM_ANGLE));
        }
      }
    }
    if (emergencyChecker(emergencyParams)) {
      break;
    };
  }
  
  return blockPositions;
}

void dealwithblock(bool(*emergencyFunc)(void*), void* emergencyParams) {
    
    const double distanceToFloorThresh = 0.08;

    coordinate robotPos = getLocation(gps);
    double bearing = getCompassBearing(getDirection(compass));

    for (char i = 0; i < 4; i++) {
    closeGripper(gripperservo); timeDelay(15, emergencyFunc, emergencyParams);
    openTrapDoor(trapdoorservo); timeDelay(15, emergencyFunc, emergencyParams);
        if (checkColour(colourSensor) == robotColour) {
            coutWithName << "I have found a block of my colour!" << endl;
            if (getWallCollisionDistance(robotPos, bearing) < eatBlockDistance + 0.05) {
                moveForward(-eatBlockDistance - 0.05, false, emergencyFunc, emergencyParams);
            }
            openGripper(gripperservo); openTrapDoor(trapdoorservo); timeDelay(15, emergencyFunc, emergencyParams);
            moveForward(eatBlockDistance, false, emergencyFunc, emergencyParams);
            if (blocks_collected < 3) {
                closeTrapDoor(trapdoorservo); timeDelay(15, emergencyFunc, emergencyParams);
                cout << getDistanceMeasurement(ds1) << endl;
                if (getDistanceMeasurement(ds1) <= distanceToFloorThresh) {
                    coutWithName << "i have choked :(" << endl;
                    openTrapDoor(trapdoorservo); timeDelay(15, emergencyFunc, emergencyParams);
                    closeGripper(gripperservo); timeDelay(15, emergencyFunc, emergencyParams);
                    moveForward(-eatBlockDistance - 0.05, false, emergencyFunc, emergencyParams);
                    openGripper(gripperservo); openTrapDoor(trapdoorservo); timeDelay(15, emergencyFunc, emergencyParams);
                    moveForward(eatBlockDistance, false, emergencyFunc, emergencyParams);
                    closeTrapDoor(trapdoorservo); timeDelay(15, emergencyFunc, emergencyParams);
                }
            }
            else { 
                closeGripper(gripperservo); timeDelay(15, emergencyFunc, emergencyParams);
            }
            blocks_collected +=1;
            coutWithName << "I think I have  " << blocks_collected << "  blocks" << endl;
            sendBlockColour(robotColour, emitter, robotColour, coordinate(0, 0));
            break;
        }
        else if (checkColour(colourSensor) != 0) {
            coutWithName << "I have found a block of the other colour!" << endl;
            openGripper(gripperservo); timeDelay(15, emergencyFunc, emergencyParams);
            closeTrapDoor(trapdoorservo); timeDelay(15, emergencyFunc, emergencyParams);
            double bearing = getCompassBearing(getDirection(compass));
            coordinate robotPos(getLocation(gps));
            coordinate newBlockPos = getBlockPositionInGrabber(robotPos, bearing);
            sendBlockColour(robotColour, emitter, (3 - robotColour), newBlockPos);
            moveForward(-0.2, false, emergencyFunc);
            break;
        }
        else {
            coutWithName << "  I can't detect Colour" << endl;
            closeTrapDoor(trapdoorservo);
            openGripper(gripperservo); timeDelay(15, emergencyFunc, emergencyParams);
            moveForward(-0.1, false, emergencyFunc, emergencyParams);
            
            if ((int) i < 3) {
              // at the last run dont move forward since the robot is moving on to the next block
              // note here we want to move to the (potentially shoved) block position so we pass true for positionIsBlock
              moveForward(0.1, true, emergencyFunc, emergencyParams);
            }
        }
    }
    sendRobotLocation(gps, robotColour, emitter);
    sendDealtwithBlock(robotColour, emitter);
}



void moveForward(double distance, bool positionIsBlock, bool (*emergencyFunc)(void*), void* emergencyParams) {
    const double* bearing = getDirection(compass);
    coordinate bearingVector(-bearing[0], bearing[2]); // I dont like using the coordinate as a vector
    coordinate targetPosition = coordinate(getLocation(gps)) + bearingVector * distance;
    updateTargetDistance(targetPosition);

    bool blockLost = false;

    while (robot->step(timeStep) != -1) {
        bearing = getDirection(compass);
        coordinate robotPos = getLocation(gps);

        tuple<double, double> motor_speeds = updatePositionalControlLoop(robotPos, bearing);
        setMotorVelocity(motors, motor_speeds);

        if (hasReachedPosition()) {
            setMotorVelocity(motors, tuple<double, double>(0.0, 0.0));
            //cout << "Arrived" << endl;
            break;
        }
        if (canUseDistanceSensor() && positionIsBlock) {
          double distance = getDistanceMeasurement(ds1);
          if (!tweakTargetDistanceFromMeasurement(robotPos, bearing, distance, 0.2)) {
            // TODO: check carefully that this works
            coutWithName << "I lost a block during tweaking" << endl;
            blockLost = true;
            coordinate relocatedTarget;
            // relocate by turning slightly
            if (relocateBlock(relocatedTarget, emergencyFunc, emergencyParams)) {
              updateTargetPosition(relocatedTarget);
              blockLost = false;
            }
            else {
              //open and close the gripper to try and relocate again
              closeGripper(gripperservo); timeDelay(15, emergencyFunc);
              openGripper(gripperservo);
              if (tweakTargetDistanceFromMeasurement(robotPos, bearing, distance, 0.2)) {
                blockLost = false;
              }
            }
          }
        }
        if (emergencyChecker(emergencyParams)) {
          setMotorVelocity(motors, tuple<double, double>(0.0, 0.0));
          break;
        };
        if (blockLost) {
          handleBlockLost();
        }
    }
}

bool moveToPosition(coordinate blockPosition, bool positionIsBlock, bool (*emergencyFunc)(void*), void* emergencyParams, bool canReverse) {
  coutWithName << "in move to position" << endl;
  isMovingToPosition = true;
  coordinate robotPos = getLocation(gps);
  coordinate nextTarget = blockPosition;
  if (positionIsBlock) {
    nextTarget = getPositionAroundBlock(blockPosition, robotPos, frontOfRobotDisplacement);
  }
  if (canReverse && !positionIsBlock) {
    updateTargetPosition(nextTarget, true);
  }
  else {
    updateTargetPosition(nextTarget);
  }
  bool hasConfirmedBlock = false;
  bool blockLost = false;

  while (robot->step(timeStep) != -1) {
    const double* bearing = getDirection(compass);
    coordinate robotPos = getLocation(gps);

    tuple<double, double> motor_speeds = updatePositionalControlLoop(robotPos, bearing);
    setMotorVelocity(motors, motor_speeds);

    if (emergencyFunc(emergencyParams)) {
      return false;
    }
    if (positionIsBlock && !hasConfirmedBlock && isMaintainingTargetBearing()) {
      blockLost = !confirmBlockPosition();
      if (blockLost) {
        coordinate relocatedTarget;
        if (relocateBlock(relocatedTarget, emergencyFunc, emergencyParams)) {
          updateTargetPosition(relocatedTarget);
          blockLost = false;
        }
      }
      hasConfirmedBlock = true;
    }
    if (canUseDistanceSensor() && positionIsBlock) {
      double distance = getDistanceMeasurement(ds1);
      if (!tweakTargetDistanceFromMeasurement(robotPos, bearing, distance, 0.2)) {
        //TODO: check carefully that this works
        coutWithName << "I lost a block during tweaking" << endl;
        blockLost = true;
        coordinate relocatedTarget;
        // relocate by turning slightly
        if (relocateBlock(relocatedTarget, emergencyFunc, emergencyParams)) {
          updateTargetPosition(relocatedTarget);
          blockLost = false;
        }
        else {
          //open and close the gripper to try and relocate again
          closeGripper(gripperservo); timeDelay(15, emergencyFunc);
          openGripper(gripperservo);
          if (tweakTargetDistanceFromMeasurement(robotPos, bearing, distance, 0.2)) {
            blockLost = false;
          }
        }
      }
    }
    if (hasReachedPosition()) {
      setMotorVelocity(motors, tuple<double, double>(0.0, 0.0));
      coutWithName << "reached end position" << endl;
      return true;
    }
    if (blockLost) {
      handleBlockLost();
      return false;
    }
  }
  isMovingToPosition = false;
}

void turnToBearing(double bearing, bool (*emergencyFunc)(void*), void* emergencyParams) {
  updateTargetBearing(bearing);

  while (robot->step(timeStep) != -1) {
    const double* bearingVector = getDirection(compass);

    tuple<double, double> motorSpeeds = updateRotationControlLoop(bearingVector);
    setMotorVelocity(motors, motorSpeeds);

    if (hasReachedTargetBearing()) {
      break;
    }
    if (emergencyChecker(emergencyParams)) {
      break;
    };
  }
}

void timeDelay(int delayLength, bool (*emergencyFunc)(void*), void* emergencyParams) {
  int j = 0;
  while (robot->step(timeStep) != -1) {
    j++;
    if (j == delayLength) {
      break;
    }
    if (emergencyFunc(emergencyParams)) {
      break;
    };
  }
}

void waitUntilCollisionEventFinished() {
  bool keepWaiting = true;
  while (robot->step(timeStep) != -1 && keepWaiting) {
    message* receivedData = receiveData(receiver);
    if (receivedData) {
      if (floor(get<0>(*receivedData) / 100) == robotColour) {
        switch (get<0>(*receivedData) % 100) {
        case(97):
          coutWithName << "Starting again, collision finished" << endl;
          keepWaiting = false;
          break;
        }
      }
    }
  }
}

bool confirmBlockPosition() {
  const double confirmBlockTolerance = 0.1;   // confirms blocks if they are within this distance of where we expect them to be
  const double* bearingVector = getDirection(compass);
  coordinate robotPos = getLocation(gps);
  double distance = getDistanceMeasurement(ds1);
  double expectedDistance = getExpectedDistanceOfBlock(robotPos, bearingVector);

  if (abs(distance - expectedDistance) > confirmBlockTolerance) {
    coutWithName << "I lost a block!" << endl;
    return false;
  }
  return true;
}

bool relocateBlock(coordinate& nextTarget, bool (*emergencyFunc)(void*), void* emergencyParams) {
  double startBearing = getCompassBearing(getDirection(compass));
  const double searchAngle = 3;
  const double wallSeparationThresh = 0.08;

  for (int i = -searchAngle; i < searchAngle; i++) {
    turnToBearing(constrainBearing(startBearing + i), emergencyFunc, emergencyParams);

    double bearing = getCompassBearing(getDirection(compass));
    coordinate robotPos = getLocation(gps);
    double distance = getDistanceMeasurement(ds1);
    if (getWallDistance(robotPos, bearing) - distance > wallSeparationThresh) {
      // the block is here, but just at the wrong distance so we update the position and proceed (the block was found)
      coutWithName << "attempting to relocate block" << endl;
      //unless the block was found at the very left of the search angle in the first run, add on the block size to get the centre of the block
      double blockBearing = (i == -searchAngle) ? bearing : bearing + (BLOCK_SIZE * RAD_TO_DEG / distance) / 2;
      coordinate newBlockPos = getBlockPositionFromAngleAndDistance(robotPos, distance, blockBearing);
      nextTarget = getPositionAroundBlock(newBlockPos, robotPos, frontOfRobotDisplacement);
      if (nextTarget.x > 1.03) nextTarget.x = 1.03;
      if (nextTarget.x < -1.03) nextTarget.x = -1.03;
      if (nextTarget.z > 1.03) nextTarget.z = 1.03;
      if (nextTarget.z < -1.03) nextTarget.z = -1.03;
      coutWithName << "my next target is  " << nextTarget << endl;
      return true;
    }
  }
  return false;
}

void handleBlockLost() {
  sendDealtwithBlock(robotColour, emitter);
  setMotorVelocity(motors, tuple<double, double>(0, 0));
}