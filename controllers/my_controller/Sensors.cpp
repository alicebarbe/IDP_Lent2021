#include <tuple>

#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/LightSensor.hpp>

#include "Sensors.hpp"

using namespace webots;
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
  ds->enable(robot->getBasicTimeStep());
  return ds;
}

/**
* Initialises and enables the light sensor. By default the sampling
* period is equal to the simualion timestep.
*
* @param robot (Robot*) a pointer to the robot object
* @param name (const char*) the name of the light sensor
* @returns ls (tuple<LightSensor*, LightSensor*>) a tuple containing pointers to the LightSensors [red_sensor, green_sensor]
*/

tuple<LightSensor*, LightSensor*> initLightSensor(Robot* robot, const char* red_sensor, const char* green_sensor) {
  LightSensor* red_ls = robot->getLightSensor(red_sensor);
  red_ls->enable(robot->getBasicTimeStep());
  LightSensor* green_ls = robot->getLightSensor(green_sensor);
  green_ls->enable(robot->getBasicTimeStep());

  return tuple<LightSensor*, LightSensor*>(red_ls, green_ls);
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
* Simulates making a colour measurement (analog input) with the
* ldr wein bridge setup. The retured value is the positive for redder 
* light and negative for greener light. The bias from the analog voltage 
* has been subtracted.
*
* @param ls (tuple<LightSensor*, LightSensor*>) the sensor to read
* @returns analogValue (int) the 10-bit voltage measurement from the 
* differential amplifier, minus the DC bias value
*/

int getLightMeasurement(tuple<LightSensor*, LightSensor*> sensors) {
  return int(get<0>(sensors)->getValue()) - int(get<1>(sensors)->getValue());
}





