
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

vector<coordinate> scanForBlocks(int timeStep);
void moveToBlock(int timeStep, coordinate blockPosition);
void sensorHeatUp(int timeStep, int warmupLength);
void sendBlockPositions(vector<coordinate> blockPositions);

int robotColour;
vector<coordinate> targetPoints;

// Webots sensors/actuators
Robot *robot = new Robot();
tuple<Motor*, Motor*> motors = initMotors(robot, "wheel1", "wheel2");
Motor* gripperservo = initServo(robot, "gripper motor");
DistanceSensor* ds1 = initDistanceSensor(robot, "ds");
GPS* gps = initGPS(robot, "gps");
Compass* compass = initCompass(robot, "compass");
tuple<LightSensor*, LightSensor*> colourSensor = initLightSensor(robot, "light_sensor_red", "light_sensor_green");
Receiver* receiver = initReceiver(robot, "receiver");
Emitter* emitter = initEmitter(robot, "emitter");

int main(int argc, char** argv) {
  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();
  sensorHeatUp(timeStep, 5);

  // identify what colour robot 

  if (getLocation(gps)[2] > 0) robotColour = RED_ROBOT; else robotColour = GREEN_ROBOT;
  sayHello(robotColour, emitter);
  sendRobotLocation(gps, robotColour, emitter);

  while (robot->step(timeStep) != -1) {
    message* receivedData = receiveData(receiver);
    if (receivedData) {
      if (floor(get<0>(*receivedData) / 10) == robotColour) {
        cout << "Robot " << robotColour << " : " << get<0>(*receivedData) << " , " << get<1>(*receivedData) << ", " << get<2>(*receivedData) << endl;
        switch (get<0>(*receivedData) % 10) {
        case(8):
          targetPoints = scanForBlocks(timeStep);
          sendRobotLocation(gps, robotColour, emitter);
          sendBlockPositions(targetPoints);
          sendFinishedScan(robotColour, emitter);
          break;
        case(0):
          cout << "Im starting to move" << endl;
          coordinate blockPosition = coordinate(get<1>(*receivedData), get<2>(*receivedData));
          moveToBlock(timeStep, blockPosition);
          break;
        }
      }
    }
  }


  /*
  vector<coordinate>targetPoints = scanForBlocks(timeStep);
  // send the blocks one by one to server

  cout << "Blocks found: " << endl;
  for (int k = 0; k < targetPoints.size(); k++) {
    cout << "( " << targetPoints[k].x << ", " << targetPoints[k].z << ") " << endl;
  }

  moveToBlocks(timeStep, targetPoints);

  // - perform simulation steps until Webots is stopping the controller
  */
  delete robot;
  return 0;
}

void sendBlockPositions(vector<coordinate> blockPositions) {
  for (auto it = blockPositions.begin(); it != blockPositions.end(); it++) {
    sendBlockLocation(*it, robotColour, emitter);
  }
}

vector<coordinate> scanForBlocks(int timeStep) {
  const int measureTimeInterval = 1;  // wait this many simulation timesteps before measuring
  const tuple<double, double> motorTurnSpeed(0.5, -0.5);
  const double blockDetectThresh = 0.05;  // detect changes in ultrasound measuruments greater than this
  const double angleToRotate = 160;

  int i = 0;

  vector<coordinate> blockPositions;

  double lastDistance = getDistanceMeasurement(ds1);
  double distance = lastDistance;
  double lastBearing = getCompassBearing(getDirection(compass));
  double bearing = lastBearing;
  double wallDistance;
  const double* robotPos;
  coordinate robotPosition;

  bool firstJump = true;
  tuple<double, double> afterLastJump;
  bool lastJumpWasFall = true;
  tuple<double, double> beforeJump, afterJump;
  bool jumpWasFall = true;

  double startBearing = bearing;
  double endBearing = (startBearing + angleToRotate) > 360 ? (startBearing + angleToRotate - 360) : startBearing + angleToRotate;
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
      robotPos = getLocation(gps);
      robotPosition = coordinate(robotPos[0], robotPos[2]);
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

        if (get<0>(beforeJump) < wallDistance - blockDetectThresh) {
          blockPositions.push_back(getBlockPosition(afterLastJump, beforeJump, lastJumpWasFall, jumpWasFall, robotPosition, ULTRASOUND_BEAM_ANGLE));
        }
      }
    }
  }
  
  return blockPositions;
}

void moveToBlock(int timeStep, coordinate blockPosition) {
  const double* pos = getLocation(gps);
  coordinate position(pos[0], pos[2]);
  coordinate nextTarget = getPositionInfrontOfBlock(blockPosition, position);
  updateTargetPosition(nextTarget);

  while (robot->step(timeStep) != -1) {
    const double* bearing = getDirection(compass);
    pos = getLocation(gps);

    position = make_tuple(pos[0], pos[2]);
    tuple<double, double> motor_speeds = moveToPosition(position, bearing);
    setMotorVelocity(motors, motor_speeds);
    if (hasFinishedTurning()) {
      double distance = getDistanceMeasurement(ds1);
      tweakBlockDistanceFromMeasurement(position, bearing, distance);
    }
    if (hasReachedPosition()) {
      break;
    }
  }
}

void sensorHeatUp(int timeStep, int warmupLength) {
  int j = 0;
  while (robot->step(timeStep) != -1) {
    j++;
    if (j == warmupLength) {
      break;
    }
  }
}