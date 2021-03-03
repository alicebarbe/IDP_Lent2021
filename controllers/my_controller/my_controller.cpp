
// File:          my_controller.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <math.h>
#include <stdlib.h>
#include <iostream>
#include <tuple>

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>

#include "CommunicationClient.hpp"
#include "Sensors.hpp"

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;

bool scan_for_blocks();
void drive_to_block(void);

bool BLOCK_DETECTED = 0;

double val1 = 0;
static double oldval1 = 0;
double blockdistance = 0;
Robot *robot = new Robot();
Motor *motor1 = robot->getMotor("wheel1");
Motor *motor2 = robot->getMotor("wheel2");
DistanceSensor* ds1 = initDistanceSensor(robot, "ds_right");

GPS* gps = initGPS(robot, "gps");
Compass* compass = initCompass(robot, "compass");
tuple<LightSensor*, LightSensor*> colour_sensor = initLightSensor(robot, "light_sensor_red", "light_sensor_green");

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.


  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  
  
  //std :: cout << "oldval1:" << oldval1 << std::endl;
      
  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
    // Read the sensors:
    oldval1 = val1;
    val1 = getDistanceMeasurement(ds1);
    //cout << getLightMeasurement(ls1) << endl;
    //cout << getlocation(gps)[0] << getlocation(gps)[1] << getlocation(gps)[2] << endl;
    cout << "x:" << getDirection(compass)[0] << "y:" << getDirection(compass)[1] << "z:" <<getDirection(compass)[2] << endl;
    // Enter here functions to read sensor data, like:
   
    //if(!BLOCK_DETECTED){
      //scan_for_blocks();}
      //else{
      //drive_to_block();}
      
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
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
    motor1->setVelocity(0);
    motor2->setVelocity(0);
    BLOCK_DETECTED = 1;
    blockdistance = val1;
  }
       
  else if(!BLOCK_DETECTED){
    motor1->setPosition(INFINITY);
    motor1->setVelocity(0.5);
    motor2->setPosition(INFINITY);
    motor2->setVelocity(-0.5);
  }

  oldval1 = val1;
  return(BLOCK_DETECTED);
}
     
void drive_to_block(){
  if(val1-blockdistance > 0.1){
    motor1->setPosition(INFINITY);
    motor2->setPosition(INFINITY);
    motor1->setVelocity(0.5);
    motor2->setVelocity(-0.5);
  }
  else if(blockdistance > 0.05){
    motor1->setPosition(INFINITY);
    motor2->setPosition(INFINITY);
    motor1->setVelocity(6);
    motor2->setVelocity(6);
    blockdistance = val1;
  }
  else {
    motor1->setVelocity(0);
    motor2->setVelocity(0);
    blockdistance = val1;
  }
}
