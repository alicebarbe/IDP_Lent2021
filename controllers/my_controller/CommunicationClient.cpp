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

coordinate getLocation(GPS* gps) {
	return coordinate(gps->getValues());
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
	const message locationMessage(robotIdentifier * 100 + 10, 0, 0);
	emitData(emitter, (const void*)&locationMessage, 20);
}


void sendRobotLocation(GPS* gps, int robotIdentifier, Emitter* emitter) {
	coordinate robotPos = getLocation(gps);
	const message locationMessage(robotIdentifier * 100 + 20, robotPos.x, robotPos.z);
	emitData(emitter,(const void*) &locationMessage, 20);
}

void sendRobotDestination(coordinate destination, int robotIdentifier, Emitter* emitter) {
	const message locationMessage(robotIdentifier * 100 + 25, destination.x, destination.z);
	emitData(emitter, (const void*)&locationMessage, 20);
}

void sendBlockLocation(coordinate blockPos, int robotIdentifier, Emitter* emitter) {
	const message locationMessage(robotIdentifier * 100 + 50, blockPos.x, blockPos.z);
	emitData(emitter, (const void*)&locationMessage, 20);
}

void sendFinishedScan(int robotIdentifier, Emitter* emitter) {
	const message locationMessage(robotIdentifier * 100 + 60, 0, 0);
	emitData(emitter, (const void*)&locationMessage, 20);
}

void sendBlockColour(int robotIdentifier, Emitter* emitter, int colour) {
	const message blockColourMessage(robotIdentifier * 100 + (colour+2)*10, 0, 0);
	emitData(emitter, (const void*)&blockColourMessage, 20);
}

void sendDealtwithBlock(int robotIdentifier, Emitter* emitter) {
	const message DealtwithBlockMessage(robotIdentifier * 100 + 70, 0, 0);
	emitData(emitter, (const void*)&DealtwithBlockMessage, 20);
}