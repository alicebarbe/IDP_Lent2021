#pragma once

#include <webots/Robot.hpp>
#include <webots/GPS.hpp>
#include <webots/Compass.hpp>

webots::GPS* initGPS(webots::Robot* robot, const char* name);
const double* getlocation(webots::GPS* gps);
webots::Compass* initCompass(webots::Robot* robot, const char* name);
const double* getDirection(webots::Compass* compass);