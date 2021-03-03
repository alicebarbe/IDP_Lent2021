#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <tuple>

#include "SimulationParameters.hpp"
#include "Motors.hpp"

using namespace webots;
using namespace std;

tuple<Motor*, Motor*> initMotors(Robot* robot, const char* left_motor, const char* right_motor) {
  Motor* motor1 = robot->getMotor(left_motor);
  Motor* motor2 = robot->getMotor(right_motor);
  motor1->setVelocity(0);
  motor2->setVelocity(0);
  motor1->setPosition(INFINITY);
  motor2->setPosition(INFINITY);
  return tuple<Motor*, Motor*>(motor1, motor2);
}

tuple<double, double> getMotorVelocity(tuple<Motor*, Motor*> motors) {
  double left_motor_v = get<0>(motors)->getVelocity();
  double right_motor_v = get<1>(motors)->getVelocity();
  return tuple<double, double>(left_motor_v, right_motor_v);
}

void setMotorVelocity(tuple<Motor*, Motor*> motors, tuple<double, double> velocities) {
  get<0>(motors)->setVelocity(get<0>(velocities));
  get<1>(motors)->setVelocity(get<1>(velocities));
}