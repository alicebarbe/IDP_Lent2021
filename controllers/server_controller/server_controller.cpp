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

#include "../my_controller/Coordinate.hpp"
#include "PathFinder/GridPathFinder.hpp"
#include "../my_controller/Utility.hpp"
#include "../my_controller/SimulationParameters.hpp"

using namespace webots;
using namespace std;

typedef tuple<int, double, double> message;
typedef tuple<double, double> coord;
typedef tuple<double, double, double, int> distance_coordinate_and_colour;
typedef vector<distance_coordinate_and_colour> coordinate_and_distance_list;

Emitter* initEmitter(Robot* server, const char* name);
Receiver* initReceiver(Robot* server, const char* name);
void emitData(Emitter* emitter, const void* data, int size);
message* receiveData(webots::Receiver* receiver);
void tell_robot_scan(int robot_identifier);
void add_block_to_list(double x_coordinate, double z_coordinate, int colour);
bool pathfind(int robot_identifier);
double distance_from_robot(int robot_identifier, double x_coordinate, double z_coordinate);
void tell_robot_go_to_block(int robot_identifier, coordinate position);
void tell_robot_go_to_position(int robot_identifier, coordinate position);
void tell_robot_go_home(int robot_identifier);
bool tell_robot_go_to_next_path_position(int robot_identifier);
void send_emergency_message(int robot_identifier);
vector<coordinate> find_path_to_block(coordinate target_block, coordinate robot_posiiton, int robot_identifier);
bool straight_path_is_clear(coordinate robot_pos, coordinate block_pos, int robot_identifier);
tuple<bool, coordinate> offsetPointAwayFromWall(coordinate blockPos, double distanceFromWallThresh, double targetOffset);
vector<coordinate> a_star_block_avoid(vector<distance_coordinate_and_colour> block_list, coordinate target_block, coordinate robot_pos, bool target_is_block, int robot_identifier = 0, bool robot_collision = false);



Robot* server = new Robot();
Receiver* receiver = initReceiver(server, "receiver");
Emitter* emitter = initEmitter(server, "emitter");
unsigned char* received_data;

coordinate_and_distance_list target_list;

bool green_going_to_target_block = false;
coordinate green_target_block;
bool red_going_to_target_block = false;
coordinate red_target_block;

coord red_position;
coord green_position;
coord green_destination;
coord red_destination;

bool green_scan_complete = false;
bool red_scan_complete = false;
bool green_robot_complete = false;
bool red_robot_complete = false;
char green_blocks_collected = 0;
char red_blocks_collected = 0;
vector<int> robot_is_home;

const double robot_half_width_clearance = 0.20;
const double radius_before_block = 0.17;

vector<coordinate> green_robot_path;
vector<coordinate> red_robot_path;


int main(int argc, char** argv) {

	// get the time step of the current world.
	int timeStep = (int)server->getBasicTimeStep();
	robot_is_home.push_back(0);
	robot_is_home.push_back(0);

	while (server->step(timeStep) != -1) {


		message* received_data = receiveData(receiver);

		if (received_data) {
			if (get<0>(*received_data) != 125 && get<0>(*received_data) != 225 && get<0>(*received_data) != 220 && get<0>(*received_data) != 120) {
				cout << "Server: " << get<0>(*received_data) << " , " << get<1>(*received_data) << ", " << get<2>(*received_data) << endl;
			}
			switch (get<0>(*received_data)) {
			case(110):tell_robot_scan(1); green_scan_complete = false; break;									//green robot has sent hello message
			case(210):tell_robot_scan(2); red_scan_complete = false; break;										//red robot has sent hello message
			case(150):
				add_block_to_list(get<1>(*received_data), get<2>(*received_data), 0); break;				//add found block location from scan to list for green
			case(250):add_block_to_list(get<1>(*received_data), get<2>(*received_data), 0); break;				//add found block location from scan to list for red
			case(120):green_position = coord(get<1>(*received_data), get<2>(*received_data)); break;	//update position of green robot
			case(220):red_position = coord(get<1>(*received_data), get<2>(*received_data)); break;		//update position of red robot

			case(125):green_destination = coord(get<1>(*received_data), get<2>(*received_data)); break;	//update destination of green robot
			case(225):red_destination = coord(get<1>(*received_data), get<2>(*received_data)); break;

			case(130):green_blocks_collected++; break;															//green robot has found a green block, Good!
			case(140):add_block_to_list(get<1>(*received_data), get<2>(*received_data), 2);					//green robot has found a red block, update colour 
				break;
			case(230):add_block_to_list(get<1>(*received_data), get<2>(*received_data), 1); 	//red robot has found a green block, update its colour in list				
				break;
			case(240):red_blocks_collected++; break;														//red robot has found a red block, Good!
			case(160):green_scan_complete = true; if (red_scan_complete) { 
				if (pathfind(1)) {
					tell_robot_go_to_next_path_position(1);
				}
				if (pathfind(2)) {
					tell_robot_go_to_next_path_position(2);
				}
			};
			break;			//green robot has finished scan, if red has too, tell them where to go
			case(260):red_scan_complete = true; if (green_scan_complete) { 
				if (pathfind(1)) {
					tell_robot_go_to_next_path_position(1);
				}
				if (pathfind(2)) {
					tell_robot_go_to_next_path_position(2);
				}
			};
			break;			//red robot has finished scan, if green has too, tell them wehre to go
			case(170):
				if (!tell_robot_go_to_next_path_position(1)) {
					green_going_to_target_block = false;
					if (green_blocks_collected != 4 && target_list.size()) {
						if (pathfind(1)) {
							tell_robot_go_to_next_path_position(1);
						}
					}
					else if (!robot_is_home[0]) tell_robot_go_home(1);
					else if (robot_is_home[1] && robot_is_home[0] && !target_list.size() && (green_blocks_collected != 4 || red_blocks_collected != 4)) {
						tell_robot_scan(1); tell_robot_scan(2);
						red_scan_complete = false; green_scan_complete = false;
					}
				}
				break;
			case(270):
				cout << "I am Red I have " << red_blocks_collected << "  blocks" << endl;
				if (!tell_robot_go_to_next_path_position(2)) {
					red_going_to_target_block = false;
					if (red_blocks_collected != 4 && target_list.size()) {
						if (pathfind(2)) {
							tell_robot_go_to_next_path_position(2);
						}
					}
					else if (!robot_is_home[1]) tell_robot_go_home(2);
					else if (robot_is_home[1] && robot_is_home[0] && !target_list.size() && (red_blocks_collected != 4 || green_blocks_collected != 4)) {
						tell_robot_scan(1); tell_robot_scan(2);
						red_scan_complete = false; green_scan_complete = false;
					}
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


bool pathfind(int robot_identifier) {
	coordinate next_target;

	if ((robot_identifier == 1 && green_blocks_collected == 4) || (robot_identifier == 2 && red_blocks_collected == 4)) {
		return false;
	}

	message new_target_message{};

	for (char i = 0; i < target_list.size(); i++)										//update distances for all blocks on list
	{
		get<0>(target_list[i]) = distance_from_robot(robot_identifier, get<1>(target_list[i]), get<2>(target_list[i]));
	}
	sort(target_list.begin(), target_list.end());										//sort list in distance from robot
	for (char j = 0; j < target_list.size(); j++) {
		if (get<3>(target_list[j]) == robot_identifier || get<3>(target_list[j]) == 0) {							//if colour is robot colour or unknown
			next_target = coordinate(get<1>(target_list[j]), get<2>(target_list[j]));		//tell robot to go there
			target_list.erase(target_list.begin() + j);									//once we have sent it as a target, remove it from list
			robot_is_home[robot_identifier - 1] = 0;
			if (robot_identifier == 1) {
				green_target_block = next_target;
				green_robot_path = find_path_to_block(next_target, green_position, robot_identifier);
				green_going_to_target_block = true;
				return true;
			}
			if (robot_identifier == 2) {
				red_target_block = next_target;
				red_robot_path = find_path_to_block(next_target, red_position, robot_identifier);
				red_going_to_target_block = true;
				return true;
			}
		}
	}
	if (robot_is_home[(robot_identifier - 1)] == 0) {
		tell_robot_go_home(robot_identifier);				//if no blocks remain on list that are unknown or belong to robot and we are not already home, go home
		return false;
	}
}

bool tell_robot_go_to_next_path_position(int robot_identifier) {
	if (robot_identifier == 1) {
		if (green_robot_path.size() > 1) {
			tell_robot_go_to_position(1, coordinate(*(green_robot_path.end()-1)));
			green_robot_path.erase((green_robot_path.end() - 1));
			return true;
		}
		else if (green_robot_path.size() == 1) {
			tell_robot_go_to_block(1, coordinate(*(green_robot_path.end() - 1)));
			green_robot_path.erase((green_robot_path.end() - 1));
			return true;
		}
	}
	else if (robot_identifier == 2) {
		if (red_robot_path.size() > 1) {
			tell_robot_go_to_position(2, coordinate(*(red_robot_path.end() - 1)));
			red_robot_path.erase((red_robot_path.end()-1));
			return true;
		}
		else if (red_robot_path.size() == 1) {
			tell_robot_go_to_block(2, coordinate(*(red_robot_path.end()-1)));
			red_robot_path.erase((red_robot_path.end()-1));
			return true;
		}
	}

	// false if the path is 0
	return false;
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

void tell_robot_go_to_block(int robot_identifier, coordinate position) {
	message go_to_position_message(robot_identifier * 100, position.x, position.z);
	emitData(emitter, (const void*)&go_to_position_message, 20);
}

void tell_robot_go_to_position(int robot_identifier, coordinate position) {
	message go_to_position_message(robot_identifier * 100 + 90, position.x, position.z);
	emitData(emitter, (const void*) &go_to_position_message, 20);
}

void send_emergency_message(int robot_identifier) {
	message new_target_message = message( 99 + robot_identifier * 100, 0, -0.4 );														//send next target green home
	emitData(emitter, (const void*) &new_target_message, 20);
}

vector<coordinate> find_path_to_block(coordinate target_block, coordinate robot_position, int robot_identifier) {
	vector<coordinate> path;
	coordinate target;
	bool target_is_via_point = false;
	tie(target_is_via_point, target) = offsetPointAwayFromWall(target_block, 0.15, 0.35);

	cout << "Trying to get to :" << target_block << endl;
	if (straight_path_is_clear(robot_position, target, robot_identifier)) {
		cout << "Path straight to block is clear" << endl;
		path.push_back(target);
	}
	else {
		cout << "Path not clear, attempting an A* pathfind" << endl;
		path = a_star_block_avoid(target_list, target, robot_position, !target_is_via_point, robot_identifier);
		if (path.size() == 0) {
			cout << "No A* solution found, ramming through" << endl;
			//ram through
			path.push_back(target);
		}
	}
	if (target_is_via_point) {
		path.insert(path.begin(), target_block);
	}
	return path;
}

vector<coordinate> a_star_block_avoid(vector<distance_coordinate_and_colour> block_list, coordinate target_block, coordinate robot_pos, bool target_is_block, int robot_identifier, bool robot_collision) {
	coordinate arena_max(ARENA_X_MAX - closestPathfindDistanceToWall, ARENA_Z_MAX - closestPathfindDistanceToWall);
	coordinate arena_min(ARENA_X_MIN + closestPathfindDistanceToWall, ARENA_Z_MIN + closestPathfindDistanceToWall);
	vector<coordinate> blocks;
	for (int i = 0; i < block_list.size(); i++) {
		blocks.push_back(coordinate(get<1>(block_list[i]), get<2>(block_list[i])));
	}
	if (green_going_to_target_block && robot_identifier == 2) {
		blocks.push_back(green_target_block);
	}
	if (red_going_to_target_block && robot_identifier == 1) {
		blocks.push_back(red_target_block);
	}

	bool has_found_path = false;
	vector<coordinate> path;
	coordinate robot_coordinate(robot_pos);
	coordinate target_displacement = target_block - robot_coordinate;
	double bearing = constrainBearing(getBearing(target_displacement * -1));

	GridPathFinder finder = GridPathFinder(arena_max, arena_min, 0.02);
	if (robot_collision) {
		if (robot_identifier == 1) {
			finder.add_circle_at_robot(red_position, 0.45);
		}
		else if (robot_identifier == 2) {
			finder.add_circle_at_robot(green_position, 0.45);
		}
	}
	finder.add_circles_at_blocks(blocks, robot_half_width_clearance);
	finder.update_children();


	if (target_is_block) {
		coordinate possible_target = target_displacement + rotateVector(coordinate(radius_before_block, 0), bearing);
		finder.print_grid(robot_pos, possible_target);

		// search += 90 degrees for any valid approach angle
		for (int i = 0; i < 90; i += 2) {
			bearing = constrainBearing(getBearing(target_displacement * -1) + i);
			possible_target = target_block + rotateVector(coordinate(radius_before_block, 0), bearing);

			if (finder.find_path(possible_target, robot_coordinate, path)) {
				has_found_path = true;
				break;
			}

			bearing = constrainBearing(getBearing(target_displacement * -1) - i);
			possible_target = target_displacement + rotateVector(coordinate(radius_before_block, 0), bearing);

			if (finder.find_path(possible_target, robot_coordinate, path)) {
				has_found_path = true;
				break;
			}
		}
	}
	else {
		if (finder.find_path(target_block, robot_coordinate, path)) {
			cout << "Found a path" << endl;
			has_found_path = true;
		}
	}

	if (has_found_path) {
		cout << "path size before simplificiation: " << path.size() << endl;

		// add the start position, then remove after simplifying otherwise the robot shivers
		path.insert(path.end() - 1, robot_pos);
		path.insert(path.begin(), target_block);
		path = finder.simplify_path(path, 0.04);
		path.pop_back();

		cout << "Path found : " << endl;
		for (int i = 0; i < path.size(); i++) {
			cout << path[i] << endl;
		}
		cout << endl;
	}

	return path;
}

bool straight_path_is_clear(coordinate robot_pos, coordinate block_pos, int robot_identifier) {
	coordinate path_bearing_vector = (block_pos - robot_pos).norm();
	coordinate perp_path_bearing_vector(-path_bearing_vector.z, path_bearing_vector.x);
	double target_distance = (block_pos - robot_pos).x * path_bearing_vector.x + (block_pos - robot_pos).z * path_bearing_vector.z;

	vector<coordinate> blocks;
	for (int i = 0; i < target_list.size(); i++) {
		blocks.push_back(coordinate(get<1>(target_list[i]), get<2>(target_list[i])));
	}
	if (green_going_to_target_block && robot_identifier == 2) {
		// add the other robot's current target, but not your own otherwise you would "collide" with it
		blocks.push_back(green_target_block);
	}
	if (red_going_to_target_block && robot_identifier == 1) {
		blocks.push_back(red_target_block);
	}

	for (int i = 0; i < blocks.size(); i++) {
		coordinate block_coordinate = blocks[i];
		if (block_coordinate == block_pos) continue;

		coordinate displacement_from_robot = block_coordinate - robot_pos;
		double distance_parallel_to_path = displacement_from_robot.x * path_bearing_vector.x + displacement_from_robot.z * path_bearing_vector.z;
		if (distance_parallel_to_path > 0 && distance_parallel_to_path < target_distance) {
			//only consider blocks infront of the robots path
			// using cross product - positive is in clockwise rotation from path
			double distance_perp_to_path = displacement_from_robot.z * path_bearing_vector.x - displacement_from_robot.x * path_bearing_vector.z;
			if (abs(distance_perp_to_path) < robot_half_width_clearance) {
				cout << "Collision with block at " << block_coordinate << endl;
				return false;
			}
		}
	}

	return true;
}

tuple<bool, coordinate> offsetPointAwayFromWall(coordinate blockPos, double distanceFromWallThresh, double targetOffset) {
	coordinate offsetPoint = blockPos;
	bool isViaPoint = false;

	if (ARENA_X_MAX - blockPos.x < distanceFromWallThresh) {
		// North wall
		offsetPoint.x = ARENA_X_MAX - targetOffset;
		isViaPoint = true;
	}
	else if (blockPos.x - ARENA_X_MIN < distanceFromWallThresh) {
		// South wall
		offsetPoint.x = ARENA_X_MIN + targetOffset;
		isViaPoint = true;
	}
	if (blockPos.z - ARENA_Z_MIN < distanceFromWallThresh) {
		// West wall
		offsetPoint.z = ARENA_Z_MIN + targetOffset;
		isViaPoint = true;
	}
	else if (ARENA_Z_MAX - blockPos.z < distanceFromWallThresh) {
		// East wall
		offsetPoint.z = ARENA_Z_MAX - targetOffset;
		isViaPoint = true;
	}
	return tuple<bool, coordinate>(isViaPoint, offsetPoint);
}
