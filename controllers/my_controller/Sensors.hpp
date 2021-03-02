
#pragma once

#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/LightSensor.hpp>

webots::DistanceSensor* initDistanceSensor(webots::Robot* robot, const char* name);
webots::LightSensor* initLightSensor(webots::Robot* robot, const char* name);

double getDistanceMeasurement(webots:: DistanceSensor* ds);
int getLightMeasurement(webots::LightSensor* ls);