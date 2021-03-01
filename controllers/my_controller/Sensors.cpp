
#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/LightSensor.hpp>

#include "Sensors.hpp"

using namespace std;

/**
* Initialises and enables the distance sensor. By default the sampling 
* period is equal to the simualion timestep.
*
* @param robot (Robot*) a pointer to the robot object
* @param name (const char*) the name of the distance sensor
* @returns ds (DistanceSensor*) a pointer to the DistanceSensor object
*/

DistanceSensor* initDistanceSensor(Robot* robot, const char* name) {
  DistanceSensor* ds = robot->getDistanceSensor(name);
  ds->enable(robot->getBasicTimeStep());  // I dont like this
  return ds;
}

/**
* Simulates making a distance measurement with the 
* ultrasonic distance sensor
* 
* @param ds (DistanceSensor*) the sensor to read
* @returns distance (int) the distance measurement in millimeters
*/

double getDistanceMeasurement(DistanceSensor* ds) {
  double measurement = ds->getValue();
  return ds->getValue();
}



