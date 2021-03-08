#pragma once

#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/LightSensor.hpp>

webots::DistanceSensor* initDistanceSensor(webots::Robot* robot, const char* name);
std::tuple<webots::LightSensor*, webots::LightSensor*> initLightSensor(webots::Robot* robot, const char* red_sensor, const char* green_sensor);

double getDistanceMeasurement(webots:: DistanceSensor* ds);
int getLightMeasurement(std::tuple<webots::LightSensor*, webots::LightSensor*> sensors);