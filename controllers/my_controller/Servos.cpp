#include "Sensors.hpp"
#include "SimulationParameters.hpp"
#include <math.h>
#include <webots/Motor.hpp>

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