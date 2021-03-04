#pragma once

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>

webots::Motor* initServo(webots::Robot*, const char* name);
void openGripper(webots::Motor* gripper);