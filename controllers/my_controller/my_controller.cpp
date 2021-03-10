
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
void turnToStartBearing(int timeStep, double startBearing);
void dealwithblock(void);
void collectblock(void);
void movePastBlock(int timeStep);

int robotColour;
vector<coordinate> targetPoints;

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
          turnToStartBearing(timeStep, -30);
          targetPoints = scanForBlocks(timeStep);
          cout << "Blocks found: " << endl;
          sendRobotLocation(gps, robotColour, emitter);
          sendBlockPositions(targetPoints);
          sendFinishedScan(robotColour, emitter);
          break;
        case(0):
          coordinate blockPosition = coordinate(get<1>(*receivedData), get<2>(*receivedData));
          moveToBlock(timeStep, blockPosition);
          dealwithblock();
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
  const double wallSeparationThresh = 0.08;   // detect blocks if they are this far from the wall
  const double angleToRotate = 200;

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

        if (get<0>(beforeJump) < wallDistance - wallSeparationThresh) {
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
      setMotorVelocity(motors, tuple<double, double>(0.0, 0.0));
      cout << "Arrived" << endl;
      break;
    }
  }
}

void turnToStartBearing(int timeStep, double shiftFromStart) {
  const double* bearingVector = getDirection(compass);
  double bearing = constrainBearing(getCompassBearing(bearingVector) + shiftFromStart);
  updateTargetBearing(bearing);

  while (robot->step(timeStep) != -1) {
    const double* bearingVector = getDirection(compass);
    tuple<double, double> motorSpeeds = turnToTargetBearing(bearingVector);
    setMotorVelocity(motors, motorSpeeds);
    if (hasReachedTargetBearing()) {
      cout << "reached starting position" << endl;
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

void dealwithblock(void) {
    gripBlock(gripperservo);
    sensorHeatUp(timeStep, 15);
    openGripper(gripperservo);
    openTrapDoor(trapdoorservo);
    sensorHeatUp(timeStep, 15);
    if (checkColour(colourSensor) == robotColour) {                                     //if block is same colour as robot
        collectblock();                                                                 //collect block
        sendBlockColour(robotColour, emitter, robotColour);                             //tell server block colour
        sendRobotLocation(gps, robotColour, emitter);
        sendDealtwithBlock(robotColour, emitter);                                       //tell server I am done dealing with this block
    }   
    else if (abs(checkColour(colourSensor) - robotColour) == 1) {                       //if block is other colour
        closeTrapDoor(trapdoorservo);
        sensorHeatUp(timeStep, 15);
        sendBlockColour(robotColour, emitter, (3 - robotColour));                       //tell robot block is other colour
        sendRobotLocation(gps, robotColour, emitter);
        sendDealtwithBlock(robotColour, emitter);                                       //tell server I am done
    }
   

}

void collectblock(void) {
    updateTargetPosition(getPositionBeyondBlock(getTargetPosition(), getDirection(compass), 0.2));
    openGripper(gripperservo);
    movePastBlock(timeStep);
    closeTrapDoor(trapdoorservo);

}

void movePastBlock(int timeStep) {
    const double* pos = getLocation(gps);
    coordinate position(pos[0], pos[2]);
    while (robot->step(timeStep) != -1) {
        const double* bearing = getDirection(compass);
        pos = getLocation(gps);

        position = make_tuple(pos[0], pos[2]);
        tuple<double, double> motor_speeds = moveToPosition(position, bearing);
        setMotorVelocity(motors, motor_speeds);
        if (hasReachedPosition()) {
            setMotorVelocity(motors, tuple<double, double>(0.0, 0.0));
            cout << "Arrived" << endl;
            break;
        }
    }
}