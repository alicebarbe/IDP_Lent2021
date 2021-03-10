#include <tuple>

#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/LightSensor.hpp>

#include "Sensors.hpp"
#include "SimulationParameters.hpp"

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
* ldr wein bridge  and comparator setup. Reference voltages (tuned 
* with a trimmer pot) are set in SimulationParameters.hpp
*
* @param ls (tuple<LightSensor*, LightSensor*>) the sensor to read
* @returns comparatorOutputs (tuple<bool, bool>) The comparator outputs 
* in the form (red, geen). Values are true when the light intensity is higher 
* than reference
*/

tuple<bool, bool> getLightMeasurement(tuple<LightSensor*, LightSensor*> sensors) {
  return tuple<bool, bool>(get<0>(sensors)->getValue() > COMPARATOR_REF_RED, get<1>(sensors)->getValue() > COMPARATOR_REF_GREEN);
}

char checkColour(tuple<LightSensor*, LightSensor*> colourSensor)
{
	if (get<0>(getLightMeasurement(colourSensor))) {
		return 2;			//RED
	}
	if (get<1>(getLightMeasurement(colourSensor))) {
		return 1;			//GREEN
	}
}



