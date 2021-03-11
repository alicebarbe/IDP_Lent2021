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
void tell_robot_go_home(int robot_identifier);
void send_emergency_message(int robot_identifier);



Robot* server = new Robot();
Receiver* receiver = initReceiver(server, "receiver");
Emitter* emitter = initEmitter(server, "emitter");
unsigned char* received_data;

coordinate_and_distance_list green_target_list;
coordinate_and_distance_list red_target_list;
coordinate red_position;
coordinate green_position;

bool green_scan_complete = false;
bool red_scan_complete = false;
bool green_robot_waiting = false;
bool red_robot_waiting = false;
char green_blocks_collected = 0;
char red_blocks_collected = 0;



int main(int argc, char** argv) {

	// get the time step of the current world.
	int timeStep = (int)server->getBasicTimeStep();


	while (server->step(timeStep) != -1) {
		

		message* received_data = receiveData(receiver);

		if (received_data) {
			cout << "Server: " << get<0>(*received_data) << " , "<< get<1>(*received_data) << ", " << get<2>(*received_data) << endl;
			switch (get<0>(*received_data)) {
			case(110):tell_robot_scan(1); green_scan_complete = false; break;								//green robot has sent hello message
			case(210):tell_robot_scan(2); red_scan_complete = false; break;									//red robot has sent hello message
			case(150):add_block_to_list(1, get<1>(*received_data), get<2>(*received_data), false); break;	//add found block location from scan to list for green
			case(250):add_block_to_list(2, get<1>(*received_data), get<2>(*received_data), false); break;	//add found block location from scan to list for red
			case(120):green_position = coordinate(get<1>(*received_data), get<2>(*received_data)); break;	//update position of green robot
			case(220):red_position = coordinate(get<1>(*received_data), get<2>(*received_data)); break;		//update position of red robot
			case(130):green_blocks_collected++;  break;															//green robot has found a green block, Good!
			case(140):add_block_to_list(2, get<1>(green_target_list[0]), get<2>(green_target_list[0]), true);	//green robot has found a red block, add it to red list
				if (red_robot_waiting) {
					pathfind(2);
				} break;		
			case(230):add_block_to_list(1, get<1>(red_target_list[0]), get<2>(red_target_list[0]), true);		//red robot has found a green block, add it to green list
				if (green_robot_waiting) {
					pathfind(1);				
				}break;		
			case(240):red_blocks_collected++; break;														//red robot has found a red block, Good!
			case(160):green_scan_complete = true; if (red_scan_complete) { pathfind(3); }; break;			//green robot has finished scan, if red has too, tell them where to go
			case(260):red_scan_complete = true; if (green_scan_complete) { pathfind(3); }; break;			//red robot has finished scan, if green has too, tell them wehre to go
			case(170):green_target_list.erase(green_target_list.begin());
				if (green_blocks_collected == 4) { tell_robot_go_home(1); break; }							//if we have all blocks, go home
				if (green_target_list.size() != 0) { pathfind(1); break; }
				else { green_robot_waiting = true; break; }												//green robot has dealt with its block, remove it from its list and tell it where to go next
			case(270):red_target_list.erase(red_target_list.begin());
				if (red_blocks_collected == 4) { tell_robot_go_home(2); break; }
				if (red_target_list.size() != 0) { pathfind(2); break; }
				else { red_robot_waiting = true; break; }
																							//red robot has dealt with its block, remove it from its list and tell it where to go next
			}
		}
		
		

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
		scan_message = { 180, 0.0 , 0.0 };								//1 means addressing green robot, 8 means telling it to scan
	else if (robot_identifier == 2)
		scan_message = { 280, 0.0, 0.0 };								//2 means addressing red robot
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
	
	message new_target_message{};
	if (robot_identifier == 1 || robot_identifier == 3) { //3 is  both red and green - this happens when first scan is done
		for (char i = 0; i < length_of_green_list; i++)										//update distances for all blocks on green list
		{
			get<0>(green_target_list[i]) = distance_from_robot(1, get<1>(green_target_list[i]), get<2>(green_target_list[i]));
		}
		sort(green_target_list.begin(), green_target_list.end());						//sort green lists into ascending order of distance
		new_target_message = { 100, get<1>(green_target_list[0]), get<2>(green_target_list[0]) }; //send next target (the closest)
		emitData(emitter, (const void*) &new_target_message, 20);
	}
	if (robot_identifier == 2 || robot_identifier == 3){
		for (char i=0; i < length_of_red_list; i++)
		{
			get<0>(red_target_list[i]) = distance_from_robot(2, get<1>(red_target_list[i]), get<2>(red_target_list[i]));
		}
		sort(red_target_list.begin(), red_target_list.end());
		new_target_message = { 200, get<1>(red_target_list[0]), get<2>(red_target_list[0]) };
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

void tell_robot_go_home(int robot_identifier) {
	if (robot_identifier == 1)
	{
		message new_target_message{};
		new_target_message = { 100, 0, -0.4 };														//send next target green home
		emitData(emitter, (const void*)&new_target_message, 20); 
	}
	else if (robot_identifier == 2)
	{
		message new_target_message{};
		new_target_message = { 200, 0, 0.4 };														//send next target green home
		emitData(emitter, (const void*)&new_target_message, 20);
	}
}

void send_emergency_message(int robot_identifier) {
	message new_target_message = message( 99 + robot_identifier * 100, 0, -0.4 );														//send next target green home
	emitData(emitter, (const void*) &new_target_message, 20);
}