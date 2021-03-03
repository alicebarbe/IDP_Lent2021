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

Compass* initCompass(Robot* robot, const char* name) {
	Compass* compass = robot->getCompass(name);
	compass->enable(robot->getBasicTimeStep());
	return compass;
}

const double* getDirection(Compass* compass) {
	return compass->getValues();
}

Receiver* initReceiver(Robot* robot, const char* name) {
	Receiver* receiver = robot->getReceiver(name);
	receiver->enable(robot->getBasicTimeStep());
	return receiver;
}

Emitter* initEmitter(Robot* robot, const char* name) {
	Emitter* emitter = robot->getEmitter(name);
	return emitter;
}

void emitData(Emitter* emitter, const void* data, int size) {
	emitter->send(data, size);
	return;
}

char* receiveData(Receiver* receiver) {
	if (receiver->getQueueLength() > 0) {					//if there is a message in receive buffer
		char* received_data = (char*) receiver->getData();		//get the message		
		receiver->nextPacket();									//move onto next message in queue	
		return received_data;
	}
	else return 0;
	
}