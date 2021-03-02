
#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/LightSensor.hpp>

#include "Sensors.hpp"

using namespace webots;

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
* Initialises and enables the light sensor. By default the sampling
* period is equal to the simualion timestep.
*
* @param robot (Robot*) a pointer to the robot object
* @param name (const char*) the name of the light sensor
* @returns ls (LightSensor*) a pointer to the DistanceSensor object
*/

LightSensor* initLightSensor(Robot* robot, const char* name) {
  LightSensor* ls = robot->getLightSensor(name);
  ls->enable(robot->getBasicTimeStep());
  return ls;
}

/**
* Simulates making a distance measurement with the 
* ultrasonic distance sensor
* 
* @param ds (DistanceSensor*) the sensor to read
* @returns distance (int) the distance measurement in millimeters
*/

double getDistanceMeasurement(DistanceSensor* ds) {
  return ds->getValue();
}

/**
* Simulates making a light measurement (analog input) with the
* ldr wein bridge setup.
*
* @param ls (LightSensor*) the sensor to read
* @returns analogValue (int) the 10-bit voltage measurement
*/

int getLightMeasurement(LightSensor* ls) {
  return ls->getValue();
}





