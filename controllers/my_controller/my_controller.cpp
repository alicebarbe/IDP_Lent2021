
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

bool emergencyChecker(void* emergencyParams);
bool bypassEmergencyChecker(void* emergencyParams);
vector<coordinate> scanForBlocks(bool (*emergencyFunc)(void*), void* emergencyParams = NULL);
void dealwithblock(bool (*emergencyFunc)(void*), void* emergencyParams = NULL);
void collectblock(bool (*emergencyFunc)(void*), void* emergencyParams = NULL);
void moveForward(double distance, bool (*emergencyFunc)(void*), void* emergencyParams = NULL);
void moveToBlock(coordinate blockPosition, bool (*emergencyFunc)(void*), void* emergencyParams = NULL);
void turnToBearing(double bearing, bool (*emergencyFunc)(void*), void* emergencyParams = NULL);
void timeDelay(int delayLength, bool (*emergencyFunc)(void*), void* emergencyParams = NULL);
void sendBlockPositions(vector<coordinate> blockPositions);

int robotColour;

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

int main(int argc, char** argv) {
  // get the time step of the current world.
  
  timeDelay(5, emergencyChecker);

  // identify what colour robot 

  if (getLocation(gps).z > 0) robotColour = RED_ROBOT; else robotColour = GREEN_ROBOT;
  sayHello(robotColour, emitter);
  sendRobotLocation(gps, robotColour, emitter);

  while (robot->step(timeStep) != -1) {
    message* receivedData = receiveData(receiver);
    if (receivedData) {
      vector<coordinate> targetPoints;
      coordinate blockPosition;

      if (floor(get<0>(*receivedData) / 100) == robotColour) {
        cout << "Robot " << robotColour << " : " << get<0>(*receivedData) << " , " << get<1>(*receivedData) << ", " << get<2>(*receivedData) << endl;
        switch (get<0>(*receivedData) % 100) {
        case(80):
          turnToBearing((robotColour == RED_ROBOT) ? 60 : 240, emergencyChecker);
          targetPoints = scanForBlocks(emergencyChecker);
          sendRobotLocation(gps, robotColour, emitter);
          sendBlockPositions(targetPoints);
          sendFinishedScan(robotColour, emitter);
          break;
        case(00):
          blockPosition = coordinate(get<1>(*receivedData), get<2>(*receivedData));
          moveToBlock(blockPosition, emergencyChecker);
          dealwithblock(emergencyChecker);
          break;
        case(99):
          cout << "something bad has happened";
          break;
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
  // exiting out does not always take you to the very top - this should likely be changed

  message* receivedData = receiveData(receiver);
  if (receivedData) {
    if (floor(get<0>(*receivedData) / 100) == robotColour) {
      switch (get<0>(*receivedData) % 100) {
      case(99):
        cout << "Robot" << robotColour << ": Something bad has happened" << endl;
        timeDelay(5, bypassEmergencyChecker);
        return false;
        break;
      }
    }
  }
}

void sendBlockPositions(vector<coordinate> blockPositions) {
  for (auto it = blockPositions.begin(); it != blockPositions.end(); it++) {
    sendBlockLocation(*it, robotColour, emitter);
  }
}

vector<coordinate> scanForBlocks(bool (*emergencyFunc)(void*), void* emergencyParams) {
  const int measureTimeInterval = 1;  // wait this many simulation timesteps before measuring
  const tuple<double, double> motorTurnSpeed(0.5, -0.5);
  const double blockDetectThresh = 0.05;  // detect changes in ultrasound measuruments greater than this
  const double wallSeparationThresh = 0.08;   // detect blocks if they are this far from the wall
  const double angleToRotate = 200;

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

void dealwithblock(bool (*emergencyFunc)(void*), void* emergencyParams) {
    gripBlock(gripperservo);
    timeDelay(20, emergencyFunc, emergencyParams);
    openGripper(gripperservo);
    openTrapDoor(trapdoorservo);
    timeDelay(15, emergencyFunc, emergencyParams);
    if (checkColour(colourSensor) == robotColour) {                                     //if block is same colour as robot
        closeGripper(gripperservo);
        closeTrapDoor(trapdoorservo);
        timeDelay(20, emergencyFunc, emergencyParams);
        moveForward(-0.15, emergencyFunc, emergencyParams);
        collectblock(emergencyFunc, emergencyParams);                                                                 //collect block
        sendBlockColour(robotColour, emitter, robotColour);                             //tell server block colour
        sendRobotLocation(gps, robotColour, emitter);
        sendDealtwithBlock(robotColour, emitter);                                       //tell server I am done dealing with this block
    }   
    else if (abs(checkColour(colourSensor) - robotColour) == 1) {                       //if block is other colour
        closeTrapDoor(trapdoorservo);
        timeDelay(15, emergencyFunc, emergencyParams);
        moveForward(-0.2, emergencyFunc);
        sendBlockColour(robotColour, emitter, (3 - robotColour));                       //tell robot block is other colour
        sendRobotLocation(gps, robotColour, emitter);
        sendDealtwithBlock(robotColour, emitter);                                       //tell server I am done
    }
}

void collectblock(bool (*emergencyFunc)(void*), void* emergencyParams) {
  openGripper(gripperservo);
  openTrapDoor(trapdoorservo);
  timeDelay(10, emergencyFunc, emergencyParams);
  moveForward(eatBlockDistance, emergencyFunc, emergencyParams);
  closeTrapDoor(trapdoorservo);
  }

void moveForward(double distance, bool (*emergencyFunc)(void*), void* emergencyParams) {
  const double* bearing = getDirection(compass);
  coordinate bearingVector(-bearing[0], bearing[2]); // I dont like using the coordinate as a vector
  coordinate targetPosition = coordinate(getLocation(gps)) + bearingVector * distance;
  updateTargetDistance(targetPosition);

  while (robot->step(timeStep) != -1) {
    bearing = getDirection(compass);
    coordinate robotPos = getLocation(gps);

    tuple<double, double> motor_speeds = updatePositionalControlLoop(robotPos, bearing);
    setMotorVelocity(motors, motor_speeds);

    if (hasReachedPosition()) {
      setMotorVelocity(motors, tuple<double, double>(0.0, 0.0));
      cout << "Arrived" << endl;
      break;
    }
    if (emergencyChecker(emergencyParams)) {
      break;
    };
  }
}

void moveToBlock(coordinate blockPosition, bool (*emergencyFunc)(void*), void* emergencyParams) {
  coordinate robotPos = getLocation(gps);
  coordinate nextTarget = getPositionAroundBlock(blockPosition, robotPos, frontOfRobotDisplacement);
  updateTargetPosition(nextTarget);

  while (robot->step(timeStep) != -1) {
    const double* bearing = getDirection(compass);
    coordinate robotPos = getLocation(gps);

    tuple<double, double> motor_speeds = updatePositionalControlLoop(robotPos, bearing);
    setMotorVelocity(motors, motor_speeds);

    if (canUseDistanceSensor()) {
      double distance = getDistanceMeasurement(ds1);
      tweakTargetDistanceFromMeasurement(robotPos, bearing, distance);
    }
    if (hasReachedPosition()) {
      setMotorVelocity(motors, tuple<double, double>(0.0, 0.0));
      break;
    }
    if (emergencyChecker(emergencyParams)) {
      break;
    };
  }
}

void turnToBearing(double bearing, bool (*emergencyFunc)(void*), void* emergencyParams) {
  updateTargetBearing(bearing);

  while (robot->step(timeStep) != -1) {
    const double* bearingVector = getDirection(compass);

    tuple<double, double> motorSpeeds = updateRotationControlLoop(bearingVector);
    setMotorVelocity(motors, motorSpeeds);

    if (hasReachedTargetBearing()) {
      cout << "reached starting position" << endl;
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
    if (emergencyChecker(emergencyParams)) {
      break;
    };
  }
}