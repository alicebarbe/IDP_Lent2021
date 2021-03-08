
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

//typedef tuple<int, double, double> message;

bool scan_for_blocks();
void drive_to_block(void);
vector<coordinate> scanForBlocks(int timeStep);
void moveToBlocks(int timeStep, vector<coordinate> blockPositions);
void sensorHeatUp(int timeStep, int warmupLength);

bool BLOCK_DETECTED = 0;

double val1 = 0;
static double oldval1 = 0;
double blockdistance = 0;

// Webots sensors/actuators
Robot *robot = new Robot();
tuple<Motor*, Motor*> motors = initMotors(robot, "wheel1", "wheel2");
Motor* gripperservo = initServo(robot, "gripper motor");
DistanceSensor* ds1 = initDistanceSensor(robot, "ds_right");
GPS* gps = initGPS(robot, "gps");
Compass* compass = initCompass(robot, "compass");
tuple<LightSensor*, LightSensor*> colourSensor = initLightSensor(robot, "light_sensor_red", "light_sensor_green");
Receiver* receiver = initReceiver(robot, "receiver");
Emitter* emitter = initEmitter(robot, "emitter");

int main(int argc, char** argv) {
  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();
  sensorHeatUp(timeStep, 5);

  vector<coordinate>targetPoints = scanForBlocks(timeStep);

  cout << "Blocks found: " << endl;
  for (int k = 0; k < targetPoints.size(); k++) {
    cout << "( " << targetPoints[k].x << ", " << targetPoints[k].z << ") " << endl;
  }

  moveToBlocks(timeStep, targetPoints);

  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
    // Read the sensors:
    oldval1 = val1;
    val1 = getDistanceMeasurement(ds1);
       
    if(!BLOCK_DETECTED){
      scan_for_blocks();
    }
    else {
      static char i = 0;
      if (i == 0) {
        message yessir{};
        yessir = { 14 ,1.1,2.4 };
        const void* alpha = &yessir;
               
        emitData(emitter, alpha, 17);//"block found", 12);
        i++;
        }
      drive_to_block();
    }
    //char* received_data = receiveData(receiver);
    /* if(received_data){
        string data_string(received_data);
        cout << data_string << endl;
    }*/

  }
  delete robot;
  return 0;
}

vector<coordinate> scanForBlocks(int timeStep) {
  const int measureTimeInterval = 1;  // wait this many simulation timesteps before measuring
  const tuple<double, double> motorTurnSpeed(0.5, -0.5);
  const double blockDetectThresh = 0.05;  // detect changes in ultrasound measuruments greater than this

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
  bool crossedNorthOnce = false;
  setMotorVelocity(motors, motorTurnSpeed);

  while (robot->step(timeStep) != -1) {
    lastDistance = distance;
    lastBearing = bearing;
    distance = getDistanceMeasurement(ds1);
    bearing = getCompassBearing(getDirection(compass));

    if (!crossedNorthOnce && bearing < startBearing) {
      crossedNorthOnce = true;
    }
    if (crossedNorthOnce && bearing > startBearing) {
      break;
    }

    i = (i + 1) % measureTimeInterval;

    if (i == 0) {
      robotPos = getlocation(gps);
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

void moveToBlocks(int timeStep, vector<coordinate> blockPositions) {
  int i = 0;
  const double* pos = getlocation(gps);
  coordinate position(pos[0], pos[2]);
  coordinate nextTarget = getPositionInfrontOfBlock(blockPositions[i], position);
  updateTargetPosition(nextTarget);

  while (robot->step(timeStep) != -1) {
    /*
    * This should go somewhere else
    bool redLDR, greenLDR;
    tie(redLDR, greenLDR) = getLightMeasurement(colourSensor);
    if (redLDR && !greenLDR) {
      cout << "Red!" << endl;
    }
    if (greenLDR && !redLDR) {
      cout << "Green!" << endl;
    }
    if (greenLDR && redLDR) {
      cout << "Overload" << endl;
    }
    */

    const double* bearing = getDirection(compass);
    pos = getlocation(gps);

    position = make_tuple(pos[0], pos[2]);
    tuple<double, double> motor_speeds = moveToPosition(position, bearing);
    setMotorVelocity(motors, motor_speeds);
    if (hasFinishedTurning()) {
      double distance = getDistanceMeasurement(ds1);
      tweakBlockDistanceFromMeasurement(position, bearing, distance);
    }
    if (hasReachedPosition()) {
      cout << "Arrived!" << endl;
      if (++i >= blockPositions.size()) {
        break;
      }
      nextTarget = getPositionInfrontOfBlock(blockPositions[i], position);
      updateTargetPosition(nextTarget);
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

bool scan_for_blocks(){ 
  static char i = 0;
  static bool SENSORHEATUP = 0;
     
  
  if (i<3){
    SENSORHEATUP = 1;
    i+=1;}
  else{SENSORHEATUP = 0;}
       

  if(std::abs(val1-oldval1) > 0.05 && !SENSORHEATUP)
  {
    setMotorVelocity(motors, tuple<double, double>(0.0, -0.0));
    BLOCK_DETECTED = 1;
    blockdistance = val1;
  }
       
  else if(!BLOCK_DETECTED){
    setMotorVelocity(motors, tuple<double, double>(0.5, -0.5));
  }

  oldval1 = val1;
  return(BLOCK_DETECTED);
}
     
void drive_to_block(){
  if(val1-blockdistance > 0.1){
    setMotorVelocity(motors, tuple<double, double>(0.5, -0.5));
  }
  else if(blockdistance > 0.05){
    setMotorVelocity(motors, tuple<double, double>(6.0, 6.0));
    blockdistance = val1;
  }
  else {
    setMotorVelocity(motors, tuple<double, double>(0.0, 0.0));
    openGripper(gripperservo);
    blockdistance = val1;
  }
}

