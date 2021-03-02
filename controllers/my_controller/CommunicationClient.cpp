#include "CommunicationClient.hpp"
#include <webots/robot.hpp>
#include <webots/GPS.hpp>

using namespace webots;

GPS* initGPS(Robot* robot, const char* name) {
	GPS* gps = robot->getGPS(name);
	gps->enable(robot->getBasicTimeStep());
	return gps;
}