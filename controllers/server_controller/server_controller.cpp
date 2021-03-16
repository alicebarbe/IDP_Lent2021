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
typedef tuple<double, double, double,int> distance_coordinate_and_colour;
typedef vector<distance_coordinate_and_colour> coordinate_and_distance_list;

Emitter* initEmitter(Robot* server, const char* name);
Receiver* initReceiver(Robot* server, const char* name);
void emitData(Emitter* emitter, const void* data, int size);
message* receiveData(webots::Receiver* receiver);
void tell_robot_scan(int robot_identifier);
void add_block_to_list(double x_coordinate, double z_coordinate, int colour);
void pathfind(int robot_identifier);
double distance_from_robot(int robot_identifier, double x_coordinate, double z_coordinate);
void tell_robot_go_home(int robot_identifier);
void send_emergency_message(int robot_identifier);



Robot* server = new Robot();
Receiver* receiver = initReceiver(server, "receiver");
Emitter* emitter = initEmitter(server, "emitter");
unsigned char* received_data;

//coordinate_and_distance_list green_target_list;
//coordinate_and_distance_list red_target_list;
coordinate_and_distance_list target_list;
coordinate red_position;
coordinate green_position;

coordinate green_destination;
coordinate red_destination;

bool green_scan_complete = false;
bool red_scan_complete = false;
bool green_robot_complete = false;
bool red_robot_complete = false;
char green_blocks_collected = 0;
char red_blocks_collected = 0;
vector<int> robot_is_home;



int main(int argc, char** argv) {

	// get the time step of the current world.
	int timeStep = (int)server->getBasicTimeStep();
	robot_is_home.push_back(0);
	robot_is_home.push_back(0);


	while (server->step(timeStep) != -1) {


		message* received_data = receiveData(receiver);

		if (received_data) {
			//if (get<0>(*received_data) != 125 && get<0>(*received_data) != 225 && get<0>(*received_data) != 220 && get<0>(*received_data) != 120) {
			if (get<0>(*received_data) != 125 && get<0>(*received_data) != 120) {
			cout << "Server: " << get<0>(*received_data) << " , " << get<1>(*received_data) << ", " << get<2>(*received_data) << endl;
			}
			switch (get<0>(*received_data)) {
			case(110):tell_robot_scan(1); green_scan_complete = false; break;									//green robot has sent hello message
			case(210):tell_robot_scan(2); red_scan_complete = false; break;										//red robot has sent hello message
			case(150):add_block_to_list(get<1>(*received_data), get<2>(*received_data), 0); break;				//add found block location from scan to list for green
			case(250):add_block_to_list(get<1>(*received_data), get<2>(*received_data), 0); break;				//add found block location from scan to list for red
			case(120):green_position = coordinate(get<1>(*received_data), get<2>(*received_data)); break;	//update position of green robot
			case(220):red_position = coordinate(get<1>(*received_data), get<2>(*received_data)); break;		//update position of red robot

			case(125):green_destination = coordinate(get<1>(*received_data), get<2>(*received_data)); break;	//update destination of green robot
			case(225):red_destination = coordinate(get<1>(*received_data), get<2>(*received_data)); break;

			case(130):green_blocks_collected++; break;															//green robot has found a green block, Good!
			case(140):add_block_to_list(get<1>(*received_data), get<2>(*received_data), 2);					//green robot has found a red block, update colour 
				break;
			case(230):add_block_to_list(get<1>(*received_data), get<2>(*received_data), 1); 	//red robot has found a green block, update its colour in list				
				break;
			case(240):red_blocks_collected++; break;														//red robot has found a red block, Good!
			case(160):green_scan_complete = true; if (red_scan_complete) { pathfind(1); pathfind(2); }; break;			//green robot has finished scan, if red has too, tell them where to go
			case(260):red_scan_complete = true; if (green_scan_complete) { pathfind(1); pathfind(2); }; break;			//red robot has finished scan, if green has too, tell them wehre to go
			case(170):
				if (green_blocks_collected != 4 && target_list.size()) pathfind(1);
				else if (!robot_is_home[0]) tell_robot_go_home(1);
				else if (robot_is_home[1] && robot_is_home[0] && !target_list.size() && (green_blocks_collected != 4 || red_blocks_collected != 4)) {
					tell_robot_scan(1); tell_robot_scan(2);
					red_scan_complete = false; green_scan_complete = false;
				}
				break;
			case(270):
				if (red_blocks_collected != 4 && target_list.size()) pathfind(2);
				else if (!robot_is_home[1]) tell_robot_go_home(2);
				else if (robot_is_home[1] && robot_is_home[0] && !target_list.size() && (red_blocks_collected != 4 || green_blocks_collected !=4)) {
					tell_robot_scan(1); tell_robot_scan(2);
					red_scan_complete = false; green_scan_complete = false;
				}
				break;




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
	scan_message = { robot_identifier * 100 + 80, green_blocks_collected, red_blocks_collected };
	emitData(emitter, (const void*) &scan_message, 20);	
}

void add_block_to_list(double x_coordinate, double z_coordinate, int colour) {			//0 is unknown colouor, 1 is green,  red is 2
	target_list.push_back(distance_coordinate_and_colour(0, x_coordinate, z_coordinate, colour));
}


void pathfind(int robot_identifier) {

	if ((robot_identifier == 1 && green_blocks_collected == 4) || (robot_identifier == 2 && red_blocks_collected == 4)) {
		return;
	}

	message new_target_message{};

	for (char i = 0; i < target_list.size(); i++)										//update distances for all blocks on list
	{
		get<0>(target_list[i]) = distance_from_robot(robot_identifier, get<1>(target_list[i]), get<2>(target_list[i]));
	}
	sort(target_list.begin(), target_list.end());										//sort list in distance from robot
	for (char j = 0; j < target_list.size(); j++) {
		if (get<3>(target_list[j]) == robot_identifier || get<3>(target_list[j]) == 0) {							//if colour is robot colour or unknown
			new_target_message = { (robot_identifier * 100), get<1>(target_list[j]), get<2>(target_list[j]) };		//tell robot to go there
			emitData(emitter, (const void*)&new_target_message, 20);
			target_list.erase(target_list.begin() + j);									//once we have sent it as a target, remove it from list
			robot_is_home[robot_identifier - 1] = 0;
			return;
		}
	}
	if (robot_is_home[(robot_identifier - 1)] == 0) {
		tell_robot_go_home(robot_identifier);				//if no blocks remain on list that are unknown or belong to robot and we are not already home, go home
	}
}

double distance_from_robot(int robot_identifier, double x_coordinate, double z_coordinate) {
	if(robot_identifier == 1)
	{
		return  sqrt(pow((x_coordinate - get<0>(green_position)),2) + pow((z_coordinate - get<1>(green_position)),2));   //use pythagoras to get distance to coordinates from green robot
	}
	if (robot_identifier == 2)
	{
		return  sqrt(pow((x_coordinate - get<0>(red_position)), 2) + pow((z_coordinate - get<1>(red_position)), 2));	// use pythagoras to get distance to coordiantes from red robot
	}
}

void tell_robot_go_home(int robot_identifier) {
	if (robot_identifier == 1)
	{
		message go_home_message{};
		go_home_message = { 190, 0.004, -0.47 };														//send next target green home
		emitData(emitter, (const void*)&go_home_message, 20);
		robot_is_home[0] = 1;
		
	}
	else if (robot_identifier == 2)
	{
		message go_home_message{};
		go_home_message = { 290, -0.004, 0.47 };														//send next target green home
		emitData(emitter, (const void*)&go_home_message, 20);
		robot_is_home[1] = 1;
		
	}
}

void send_emergency_message(int robot_identifier) {
	message new_target_message = message( 99 + robot_identifier * 100, 0, -0.4 );														//send next target green home
	emitData(emitter, (const void*) &new_target_message, 20);
}