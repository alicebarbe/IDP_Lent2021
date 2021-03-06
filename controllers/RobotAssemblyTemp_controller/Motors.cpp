#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <tuple>

//#include "SimulationParameters.hpp"
#include "Motors.hpp"

using namespace webots;
using namespace std;

tuple<Motor*, Motor*, Motor*, Motor*> initMotors(Robot* robot, const char* left_motor, const char* right_motor, const char* trap_motor, const char* flipper_motor) {
  Motor* motor1 = robot->getMotor(left_motor);
  Motor* motor2 = robot->getMotor(right_motor);
  Motor* motor3 = robot->getMotor(trap_motor);
  Motor* motor4 = robot->getMotor(flipper_motor);
  motor1->setVelocity(0);
  motor2->setVelocity(0);
  motor3->setVelocity(0);
  motor4->setVelocity(0);
  motor1->setPosition(INFINITY);
  motor2->setPosition(INFINITY);
  motor3->setPosition(INFINITY);
  motor4->setPosition(INFINITY);
  return tuple<Motor*, Motor*, Motor*, Motor*>(motor1, motor2, motor3, motor4);
}

tuple<double, double, double, double> getMotorVelocity(tuple<Motor*, Motor*, Motor*, Motor*> motors) {
  double left_motor_v = get<0>(motors)->getVelocity();
  double right_motor_v = get<1>(motors)->getVelocity();
  double trap_motor_v = get<2>(motors)->getVelocity();
  double flipper_motor_v = get<3>(motors)->getVelocity();
  return tuple<double, double, double, double>(left_motor_v, right_motor_v, trap_motor_v, flipper_motor_v);
}

void setMotorVelocity(tuple<Motor*, Motor*, Motor*, Motor*> motors, tuple<double, double, double, double> velocities) {
  get<0>(motors)->setVelocity(get<0>(velocities));
  get<1>(motors)->setVelocity(get<1>(velocities));
  get<2>(motors)->setVelocity(get<2>(velocities));
  get<3>(motors)->setVelocity(get<3>(velocities));
}