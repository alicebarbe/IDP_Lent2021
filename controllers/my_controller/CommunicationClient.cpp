#include "CommunicationClient.hpp"
#include <webots/robot.hpp>
#include <webots/GPS.hpp>

using namespace webots;


GPS* initGPS(Robot* robot, const char* name) {							//initialises GPS module for the robot
	GPS* gps = robot->getGPS(name);
	gps->enable(robot->getBasicTimeStep());								//sets default sampling period to simulation timestep		
	return gps;															//returns gps,  a pointer to GPS
}

const double* getlocation(GPS* gps) {
	return gps->getValues();
}