// File:          server_controller.cpp
// Date:
// Description:
// Author:
// Modifications:


#include <webots/Robot.hpp>
#include <webots/Receiver.hpp>
#include <webots/Emitter.hpp>
#include <tuple>



using namespace webots;
using namespace std;
typedef tuple<int, double, double> message;

Emitter* initEmitter(Robot* server, const char* name);
Receiver* initReceiver(Robot* server, const char* name);
void emitData(Emitter* emitter, const void* data, int size);
message* receiveData(webots::Receiver* receiver);

Robot* server = new Robot();
Receiver* receiver = initReceiver(server, "receiver");
Emitter* emitter = initEmitter(server, "emitter");
unsigned char* received_data;


int main(int argc, char** argv) {



	// get the time step of the current world.
	int timeStep = (int)server->getBasicTimeStep();




	while (server->step(timeStep) != -1) {
		message* received_data = receiveData(receiver);
		
		

		if (received_data) {
		
			cout << "server receives:  " << get<0>(*received_data) << "  " << get<1>(*received_data) << "  " << get<2>(*received_data) << endl;
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

message* receiveData(Receiver* receiver) {
	if (receiver->getQueueLength() > 0) {					//if there is a message in receive buffer
		message* received_data = (message*) receiver->getData();		//get the message		
		receiver->nextPacket();									//move onto next message in queue	
		return received_data;
	}
	else return 0;

}

