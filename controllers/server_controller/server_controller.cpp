// File:          server_controller.cpp
// Date:
// Description:
// Author:
// Modifications:


#include <webots/Robot.hpp>
#include <webots/Receiver.hpp>
#include <webots/Emitter.hpp>


using namespace webots;
using namespace std;

Emitter* initEmitter(Robot* server, const char* name);
Receiver* initReceiver(Robot* server, const char* name);
void emitData(Emitter* emitter, const void* data, int size);
char* receiveData(Receiver* receiver);

Robot* server = new Robot();
Receiver* receiver = initReceiver(server, "receiver");
Emitter* emitter = initEmitter(server, "emitter");


int main(int argc, char** argv) {



	// get the time step of the current world.
	int timeStep = (int)server->getBasicTimeStep();




	while (server->step(timeStep) != -1) {
		char* received_data = receiveData(receiver);

		if (received_data) {
			string data_string(received_data);
			cout << "server receives" << data_string << endl;
		};


	}
		delete server;
		return 0;
	
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
		char* received_data = (char*)receiver->getData();		//get the message		
		receiver->nextPacket();									//move onto next message in queue	
		return received_data;
	}
	else return 0;
}

