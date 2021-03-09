#include "CommunicationClient.hpp"
#include "Coordinate.hpp"
#include <webots/robot.hpp>
#include <webots/GPS.hpp>
#include <tuple>

using namespace webots;
using namespace std;


GPS* initGPS(Robot* robot, const char* name) {							//initialises GPS module for the robot
	GPS* gps = robot->getGPS(name);
	gps->enable(robot->getBasicTimeStep());								//sets default sampling period to simulation timestep		
	return gps;															//returns gps,  a pointer to GPS
}

const double* getLocation(GPS* gps) {
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

message* receiveData(Receiver* receiver) {
	if (receiver->getQueueLength() > 0) {					//if there is a message in receive buffer
		message* received_data = (message*) receiver->getData();		//get the message		
		receiver->nextPacket();									//move onto next message in queue	
		return received_data;
	}
	else return 0;
	
}

void sayHello(int robotIdentifier, Emitter* emitter) {
	const message locationMessage(robotIdentifier * 10 + 1, 0, 0);
	emitData(emitter, (const void*)&locationMessage, 20);
}


void sendRobotLocation(GPS* gps, int robotIdentifier, Emitter* emitter) {
	const double* robotPos = getLocation(gps);
	const message locationMessage(robotIdentifier * 10 + 2, robotPos[0], robotPos[2]);
	emitData(emitter,(const void*) &locationMessage, 20);
}

void sendBlockLocation(coordinate blockPos, int robotIdentifier, Emitter* emitter) {
	const message locationMessage(robotIdentifier * 10 + 5, blockPos.x, blockPos.z);
	emitData(emitter, (const void*)&locationMessage, 20);
}

void sendFinishedScan(int robotIdentifier, Emitter* emitter) {
	const message locationMessage(robotIdentifier * 10 + 6, 0, 0);
	emitData(emitter, (const void*)&locationMessage, 20);
}