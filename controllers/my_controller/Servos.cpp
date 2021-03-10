#include "Sensors.hpp"
#include "SimulationParameters.hpp"
#include <math.h>
#include <webots/Motor.hpp>
#include "Servos.hpp"

using namespace webots;
using namespace std;

Motor* initServo(Robot* robot, const char* name) {
	Motor* servo = robot->getMotor(name);
	return servo;
}

void openGripper(Motor* gripper) {
	gripper->setPosition(1.5707);
	return;
}

void closeGripper(Motor* gripper) {
	gripper->setPosition(-2.5);
}

void openTrapDoor(Motor* trapdoor) {
	trapdoor->setPosition(-1.5707);
}

void closeTrapDoor(Motor* trapdoor) {
	trapdoor->setPosition(0);
}