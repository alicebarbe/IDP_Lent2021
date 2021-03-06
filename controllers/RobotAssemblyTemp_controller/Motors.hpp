#pragma once

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <tuple>

std::tuple<webots::Motor*, webots::Motor*, webots::Motor*, webots::Motor*> initMotors(webots::Robot* robot, const char* left_motor, const char* right_motor, const char* trap_motor, const char* flipper_motor);
std::tuple<double, double, double, double> getMotorVelocity(std::tuple<webots::Motor*, webots::Motor*, webots::Motor*, webots::Motor*> motors);
void setMotorVelocity(std::tuple<webots::Motor*, webots::Motor*, webots::Motor*, webots::Motor*> motors, std::tuple<double, double, double, double>);
