// File:          server_controller.cpp
// Date:
// Description:
// Author:
// Modifications:


#include <webots/Robot.hpp>
#include <webots/Receiver.hpp>
#include <webots/Emitter.hpp>
#include <tuple>
#include <math.h>
#include <algorithm>


using namespace webots;
using namespace std;

typedef tuple<int, double, double> message;
typedef tuple<double, double> coordinate;
typedef tuple<double, double, double> distance_and_coordinate;
typedef vector<distance_and_coordinate> coordinate_and_distance_list;

Emitter* initEmitter(Robot* server, const char* name);
Receiver* initReceiver(Robot* server, const char* name);
void emitData(Emitter* emitter, const void* data, int size);
message* receiveData(webots::Receiver* receiver);
void tell_robot_scan(int robot_identifier);
void add_block_to_list(int identifier, double x_coordinate, double z_coordinate, bool known_colour);
void pathfind(int robot_identifier);
double distance_from_robot(int robot_identifier, double x_coordinate, double z_coordinate);



Robot* server = new Robot();
Receiver* receiver = initReceiver(server, "receiver");
Emitter* emitter = initEmitter(server, "emitter");
unsigned char* received_data;

coordinate_and_distance_list green_target_list;
coordinate_and_distance_list red_target_list;
coordinate red_position;
coordinate green_position;



int main(int argc, char** argv) {

	


	// get the time step of the current world.
	int timeStep = (int)server->getBasicTimeStep();




	while (server->step(timeStep) != -1) {
		

		message* received_data = receiveData(receiver);

		if (received_data) {
			cout << "Server: " << get<0>(*received_data) << " , "<< get<1>(*received_data) << ", " << get<2>(*received_data) << endl;
			switch (get<0>(*received_data)) {
			case(11):tell_robot_scan(1); break;																//green robot has sent hello message
			case(21):tell_robot_scan(2); break;																//red robot has sent hello message
			case(15):add_block_to_list(1, get<1>(*received_data), get<2>(*received_data), false); break;			//add found block location from scan to list for green
			case(25):add_block_to_list(2, get<1>(*received_data), get<2>(*received_data), false); break;			//add found block location from scan to list for red
			case(12):green_position = coordinate(get<1>(*received_data), get<2>(*received_data)); break;	//update position of green robot
			case(22):red_position = coordinate(get<1>(*received_data), get<2>(*received_data)); break;		//update position of red robot
			case(13):break;																					//green robot has found a green block, Good!
			case(14):add_block_to_list(2, get<1>(*received_data), get<2>(*received_data), true); break;			//green robot has found a red block, add it to red list
			case(23):add_block_to_list(1, get<1>(*received_data), get<2>(*received_data), true); break;			//red robot has found a green block, add it to green list
			case(24):break;																					//red robot has found a red block, Good!
			case(16):pathfind(1); break;																	//green robot has finished scan, tell it where to go next
			case(26):pathfind(2); break;																	//red robot has finished scan, tell it wehre to go next
			case(17):green_target_list.erase(green_target_list.begin()); pathfind(1); break;				//green robot has dealt with its block, remove it from its list and tell it where to go next
			case(27):red_target_list.erase(red_target_list.begin()); pathfind(2); break;					//red robot has dealt with its block, remove it from its list and tell it where to go next
			}
		}
	
		

		//if (received_data) {
		
			//cout << "server receives:  " << get<0>(*received_data) << "  " << get<1>(*received_data) << "  " << get<2>(*received_data) << endl;
		//};

		
		

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

void tell_robot_scan(int robot_identifier) {							//outputs a message telling the green robot to scan for blocks
	message scan_message{};
	if(robot_identifier == 1)
		scan_message = { 18, 0.0 , 0.0 };								//1 means addressing green robot, 8 means telling it to scan
	else if (robot_identifier == 2)
		scan_message = { 28, 0.0, 0.0 };								//2 means addressing red robot
	emitData(emitter, (const void*) &scan_message, 20);	
}

void add_block_to_list(int robot_identifier, double x_coordinate, double z_coordinate, bool known_colour) {		//function recieves location of a block, and adds it to the relevant list
	if (robot_identifier == 1){				//if the green robot found it during scanning, or if the red robot found a block to be green
		if (green_target_list.size() < 4 || known_colour == true) {			//if the green list isn't full or if we know the block is green
			green_target_list.push_back(distance_and_coordinate(distance_from_robot(1, x_coordinate, z_coordinate),x_coordinate, z_coordinate));		
		}
		else                         //if the green list already has 4 blocks and we don't know the blocks colour add to red list
			red_target_list.push_back(distance_and_coordinate(distance_from_robot(2, x_coordinate, z_coordinate), x_coordinate, z_coordinate));
	}
	else if (robot_identifier == 2) {			//if the red robot found it during scanning, or if the green robot found a red block
		if (red_target_list.size() < 4 || known_colour == true) {
			red_target_list.push_back(distance_and_coordinate(distance_from_robot(2, x_coordinate, z_coordinate), x_coordinate, z_coordinate));
		}
		else
			green_target_list.push_back(distance_and_coordinate(distance_from_robot(1, x_coordinate, z_coordinate), x_coordinate, z_coordinate));
	}
}

void pathfind(int robot_identifier) {
	int length_of_red_list = red_target_list.size();
	int length_of_green_list = green_target_list.size();
	char i = 0;
	message new_target_message{};
	if (robot_identifier == 1) {
		for (i; i < length_of_green_list; i++)										//update distances for all blocks on green list
		{
			get<0>(green_target_list[i]) = distance_from_robot(1, get<1>(green_target_list[i]), get<2>(green_target_list[i]));
		}
		sort(green_target_list.begin(), green_target_list.end());						//sort green lists into ascending order of distance
		new_target_message = { 10, get<1>(green_target_list[0]), get<2>(green_target_list[0]) }; //send next target (the closest)
		emitData(emitter, (const void*) &new_target_message, 20);
	}
	else if (robot_identifier == 2) {
		for (i; i < length_of_red_list; i++)
		{
			get<0>(red_target_list[i]) = distance_from_robot(2, get<1>(red_target_list[i]), get<2>(red_target_list[i]));
		}
		sort(red_target_list.begin(), red_target_list.end());
		new_target_message = { 20, get<1>(red_target_list[0]), get<2>(red_target_list[0]) };
		emitData(emitter, (const void*) &new_target_message, 20);
	}

	
}

double distance_from_robot(int robot_identifier, double x_coordinate, double z_coordinate) {
	if(robot_identifier == 1)
	{
		return  sqrt(pow((x_coordinate - get<0>(green_position)),2) + pow((z_coordinate - get<1>(green_position)),2));   //use pythagoras to get distance to coordinates from greeen robot
	}
	if (robot_identifier == 2)
	{
		return  sqrt(pow((x_coordinate - get<0>(red_position)), 2) + pow((z_coordinate - get<1>(red_position)), 2));	// use pythagoras to get distance to coordiantes from red robot
	}
}
