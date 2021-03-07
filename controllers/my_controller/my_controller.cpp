
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
#include "Sensors.hpp"
#include "Movement.hpp"
#include "Motors.hpp"
#include "Servos.hpp"

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;

//typedef tuple<int, double, double> message;

bool scan_for_blocks();
void drive_to_block(void);
vector<tuple<double, double>> scanForBlocks(int timeStep);
tuple<double, double> getBlockPosition(double angle, double distance, const double* robot_pos, const tuple<double, double> sensorDisplacement);
double getWallDistance(const double* robot_pos, double angle, const tuple<double, double> sensorDisplacement);
tuple<double, double> rotateVector(const tuple<double, double> vector, double angle);
std::tuple<double, double> getPositionInfrontOfBlock(std::tuple<double, double> blockPosition, std::tuple<double, double> robotPosition, std::tuple<double, double> grabberDisplacement);
void moveToBlocks(int timeStep, vector<tuple<double, double>> blockPositions);

bool BLOCK_DETECTED = 0;

double val1 = 0;
static double oldval1 = 0;
double blockdistance = 0;
Robot *robot = new Robot();
tuple<Motor*, Motor*> motors = initMotors(robot, "wheel1", "wheel2");
DistanceSensor* ds1 = initDistanceSensor(robot, "ds_right");
Motor* gripperservo = initServo(robot, "gripper motor");

GPS* gps = initGPS(robot, "gps");
Compass* compass = initCompass(robot, "compass");
tuple<LightSensor*, LightSensor*> colour_sensor = initLightSensor(robot, "light_sensor_red", "light_sensor_green");
Receiver* receiver = initReceiver(robot, "receiver");
Emitter* emitter = initEmitter(robot, "emitter");

tuple<double, double> frontOfRobot(0.1, 0.025);
tuple<double, double> distanceSensorDisplacement(0.05, 0.0);

int main(int argc, char** argv) {
  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();
  int j = 0;
  while (robot->step(timeStep) != -1) {
    j++;
    if (j == 10) {
      break;
    }
  }

  vector<tuple<double, double>>targetPoints = scanForBlocks(timeStep);

  cout << "Blocks found: " << endl;
  for (int k = 0; k < targetPoints.size(); k++) {
    cout << "( " << get<0>(targetPoints[k]) << ", " << get<1>(targetPoints[k]) << ") " << endl;
  }

  moveToBlocks(timeStep, targetPoints);

  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
    // Read the sensors:
    oldval1 = val1;
    val1 = getDistanceMeasurement(ds1);
        
    //cout << getLightMeasurement(ls1) << endl;
    //cout << getlocation(gps)[0] << getlocation(gps)[1] << getlocation(gps)[2] << endl;
    //cout << "x:" << getDirection(compass)[0] << "y:" << getDirection(compass)[1] << "z:" <<getDirection(compass)[2] << endl;
    // Enter here functions to read sensor data, like:

        
        if(!BLOCK_DETECTED){
          scan_for_blocks();}
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

    };
      

  delete robot;
  return 0;
}

vector< tuple<double, double> > scanForBlocks(int timeStep) {
  const int measureTimeInterval = 1;  // wait this many simulation timesteps before measuring
  const tuple<double, double> motorTurnSpeed(0.5, -0.5);
  const double blockDetectThresh = 0.05;
  const double sensorBeamAngle = 0;
  const double blockSize = 0.05;

  int i = 0;

  vector<double> distances;
  vector<double> bearings;
  vector< tuple<double, double>> blockPositions;

  bool detectingBlock = false;
  bool facingBlock = false;

  double lastDistance = getDistanceMeasurement(ds1);
  double distance = lastDistance;
  double lastBearing = getCompassBearing(getDirection(compass));
  double bearing = lastBearing;
  double wallDistance;

  bool firstJump = true;
  tuple<double, double> beforeLastJump, afterLastJump;
  bool lastJumpWasFall = true;
  tuple<double, double> beforeJump, afterJump;
  bool jumpWasFall = true;

  const double* robotPos;

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
    if (crossedNorthOnce && !facingBlock && bearing > startBearing) {
      break;
    }

    i = (i + 1) % measureTimeInterval;

    if (i == 0) {
      robotPos = getlocation(gps);
      wallDistance = getWallDistance(robotPos, bearing, distanceSensorDisplacement);

      // check for a sudden fall in distance
      if (distance - lastDistance < -blockDetectThresh) {
        cout << "fall in distance" << endl;
        if (firstJump) {
          beforeJump = make_tuple(lastDistance, lastBearing);
          afterJump = make_tuple(distance, bearing);
          jumpWasFall = true;

          beforeLastJump = beforeJump;
          afterLastJump = afterJump;
          lastJumpWasFall = jumpWasFall;

          firstJump = false;
          continue;
        }

        beforeLastJump = beforeJump;
        afterLastJump = afterJump;
        lastJumpWasFall = jumpWasFall;

        beforeJump = make_tuple(lastDistance, lastBearing);
        afterJump = make_tuple(distance, bearing);
        jumpWasFall = true;
          
        if (get<0>(beforeJump) < wallDistance - blockDetectThresh) {
          if (abs(getBearingDifference(get<1>(afterLastJump), get<1>(beforeJump))) >= sensorBeamAngle) {
            // block is not being obscured by anything else
            double blockAvgDistance = (get<0>(afterLastJump) + get<0>(beforeJump)) / 2;
            double blockAvgAngle = (get<1>(afterLastJump) + get<1>(beforeJump)) / 2;

            blockPositions.push_back(getBlockPosition(blockAvgAngle, blockAvgDistance, robotPos, distanceSensorDisplacement));
          }
          else if (lastJumpWasFall) {
            // block partially obscured by something infront of it at the new jump side
            double blockAvgDistance = (get<0>(afterLastJump) + get<0>(beforeJump)) / 2;
            double blockAvgAngle = get<1>(afterLastJump) + (sensorBeamAngle + (blockSize * 180) / (blockAvgDistance * M_PI)) / 2;

            blockPositions.push_back(getBlockPosition(blockAvgAngle, blockAvgDistance, robotPos, distanceSensorDisplacement));
          }
          else {
            // in this case we still have some information on the block, might be worth sending
            cout << "Not possible to accurately find block, ignoring" << endl;
          }
        }
      }

      // check for a sudden rise in distance
      if (distance - lastDistance > blockDetectThresh) {
        cout << "rise in distance" << endl;
        if (firstJump) {
          beforeJump = make_tuple(lastDistance, lastBearing);
          afterJump = make_tuple(distance, bearing);
          jumpWasFall = false;

          beforeLastJump = beforeJump;
          afterLastJump = afterJump;
          lastJumpWasFall = jumpWasFall;
          firstJump = false;
          continue;
        }

        beforeLastJump = beforeJump;
        afterLastJump = afterJump;
        lastJumpWasFall = jumpWasFall;

        beforeJump = make_tuple(lastDistance, lastBearing);
        afterJump = make_tuple(distance, bearing);
        jumpWasFall = false;

        if (get<0>(beforeJump) < wallDistance - blockDetectThresh) {
          if (abs(getBearingDifference(get<1>(afterLastJump), get<1>(beforeJump))) >= sensorBeamAngle) {
            // block is not being obscured by anything else
            double blockAvgDistance = (get<0>(afterLastJump) + get<0>(beforeJump)) / 2;
            double blockAvgAngle = (get<1>(afterLastJump) + get<1>(beforeJump)) / 2;

            blockPositions.push_back(getBlockPosition(blockAvgAngle, blockAvgDistance, robotPos, distanceSensorDisplacement));
          }
          else if (!lastJumpWasFall) {
            // block partially obscured by something infront of it at the last jump side
            double blockAvgDistance = (get<0>(afterLastJump) + get<0>(beforeJump)) / 2;
            double blockAvgAngle = get<1>(beforeJump) - (sensorBeamAngle + (blockSize * 180) / (blockAvgDistance * M_PI)) / 2;

            blockPositions.push_back(getBlockPosition(blockAvgAngle, blockAvgDistance, robotPos, distanceSensorDisplacement));
          }
          else {
            cout << "Measurement too small, ignoring" << endl;
          }
        }
      }
    }
  }
  
  return blockPositions;
}

void moveToBlocks(int timeStep, vector<tuple<double, double>> blockPositions) {
  int i = 0;
  const double* pos = getlocation(gps);
  tuple<double, double> position(pos[0], pos[2]);
  tuple<double, double> nextTarget = getPositionInfrontOfBlock(blockPositions[i], position, frontOfRobot);
  updateTargetPosition(nextTarget);

  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
    // Read the sensors:
    const double* bearing = getDirection(compass);
    pos = getlocation(gps);

    position = make_tuple(pos[0], pos[2]);
    tuple<double, double> motor_speeds = moveToPosition(position, bearing);
    setMotorVelocity(motors, motor_speeds);
    if (hasReachedPosition()) {
      cout << "Arrived!" << endl;
      i = (i + 1);
      if (i >= blockPositions.size()) {
        break;
      }
      nextTarget = getPositionInfrontOfBlock(blockPositions[i], position, frontOfRobot);
      updateTargetPosition(nextTarget);
    }
  }
}

tuple<double, double> getBlockPosition(double angle, double distance, const double* robot_pos, const tuple<double, double> sensorDisplacement) {
  tuple<double, double> rotatedSensorDisp = rotateVector(sensorDisplacement, angle);
  double block_x = robot_pos[0] + get<0>(rotatedSensorDisp) + distance * cos(angle * M_PI / 180.0);
  double block_z = robot_pos[2] + get<1>(rotatedSensorDisp) + distance * sin(angle * M_PI / 180.0);
  return tuple<double, double>(block_x, block_z);
}

tuple<double, double> getPositionInfrontOfBlock(tuple<double, double> blockPosition, tuple<double, double> robotPosition, tuple<double, double> grabberDisplacement) {
  tuple<double, double> displacement(get<0>(blockPosition) - get<0>(robotPosition), get<1>(blockPosition) - get<1>(robotPosition));
  double target_bearing = getBearing(displacement);

  tuple<double, double> rotatedGrabberDisp = rotateVector(grabberDisplacement, target_bearing);
  return tuple<double, double>(get<0>(blockPosition) - get<0>(rotatedGrabberDisp), get<1>(blockPosition) - get<1>(rotatedGrabberDisp));
}

double getWallDistance(const double* robot_pos, double angle, const tuple<double, double> sensorDisplacement) {
  double boundXPos, boundXNeg, boundZPos, boundZNeg;
  double radAngle = angle * M_PI / 180.0;
  tuple<double, double> rotatedSensorDisp = rotateVector(sensorDisplacement, angle);

  boundXPos = (1.2 - robot_pos[0] - get<0>(rotatedSensorDisp)) / cos(radAngle);
  boundXNeg = (-1.2 - robot_pos[0] - get<0>(rotatedSensorDisp)) / cos(radAngle);

  boundZPos = (1.2 - robot_pos[2] - get<1>(rotatedSensorDisp)) / sin(radAngle);
  boundZNeg = (-1.2 - robot_pos[2] - get<1>(rotatedSensorDisp)) / sin(radAngle);

  return min(max(boundXPos, boundXNeg), max(boundZNeg, boundZPos));
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

