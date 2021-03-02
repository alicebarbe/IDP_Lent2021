#pragma once

#include <webots/Robot.hpp>
#include <webots/GPS.hpp>

webots::GPS* initGPS(webots::Robot* robot, const char* name);