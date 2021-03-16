#include "PathFinder.h"
#include "AStar.h"
#include "../../my_controller/Coordinate.hpp"
#include "GridPathFinder.hpp"

#include <chrono>
#include <iostream>
#include <vector>
#include <algorithm>

using namespace std;

/*
Create our own class deriving AStarNode.
This is needed for applying A* Algorithm to our squares.
*/

Square::Square()
{}

Square::~Square()
{}

void Square::setType(const bool type)
{
	m_type = type;
}

bool Square::getType() const
{
	return m_type;
}

// A diagonal move has a sqrt(2) cost, 1 otherwise
float Square::localDistanceTo(AStarNode* node)
{
	if(node->getX() != m_x && node->getY() != m_y)
		return 1.41421356237f;
	else
		return 1.0f;
}

float Square::distanceTo(AStarNode* node) const
{
	int newX = m_x - node->getX(), newY = m_y - node->getY();
	return sqrtf( static_cast<float>(newX*newX + newY*newY) );
}


/*
vvvv GridPathFinder Class vvvv
*/


GridPathFinder::GridPathFinder(coordinate a_max, coordinate a_min, double sq_size) {
	arena_max = a_max;
	arena_min = a_min;
	square_size = sq_size;

	width = (int)floor((arena_max.z - arena_min.z) / square_size);
	height = (int)floor((arena_max.x - arena_min.x) / square_size);

	make_point_grid();
	update_children();
}

GridPathFinder::~GridPathFinder() {
	delete_point_grid();
}


void GridPathFinder::make_point_grid() {
	squares = new Square * [height];
	for (unsigned short int x = 0; x < height; ++x)
	{
		squares[x] = new Square[height];
		for (unsigned short int y = 0; y < width; ++y)
		{
			squares[x][y].setPosition(x, y);
			squares[x][y].setType(true);  // all squares are viable
		}
	}
}

void GridPathFinder::add_circles_at_blocks(vector<coordinate> blocks, double radius) {
	for (int x = 0; x < height; x++) {
		for (int y = 0; y < width; y++) {
			for (int c = 0; c < blocks.size(); c++) {
				coordinate square_disp = coordinate_from_square_pos(x, y) - blocks[c];
				if (sqrt(pow(square_disp.x, 2) + pow(square_disp.z, 2)) < radius) {
					squares[x][y].setType(false);
				}
			}
		}
	}
}

void GridPathFinder::add_circle_at_robot(coordinate robot_position, double radius) {
	for (int x = 0; x < height; x++) {
		for (int y = 0; y < width; y++) {
			coordinate square_disp = coordinate_from_square_pos(x, y) - robot_position;
			if (sqrt(pow(square_disp.x, 2) + pow(square_disp.z, 2)) < radius) {
				squares[x][y].setType(false);
			}
		}
	}
}

/*
vector<tuple<int, int>> GridPathFinder::get_squares_at_distance_from_point(coordinate block, double radius, double tol) {
	vector<tuple<int, int>> possible_targets;

	for (int x = 0; x < height; x++) {
		for (int y = 0; y < width; y++) {
			coordinate square_disp = coordinate_from_square_pos(x, y) - block;
			if (squares[x][y].getType() && abs(sqrt(pow(square_disp.x, 2) + pow(square_disp.z, 2)) - radius) < tol) {
				possible_targets.push_back(tuple<int, int>(x, y));
			}
		}
	}

	return possible_targets;
}
*/

void GridPathFinder::update_children() {
	// this links the nodes together in a grid

	int newX, newY;
	Square* aChild;
	for (int x = 0; x < width; ++x) {
		for (int y = 0; y < height; ++y) // traverse all squares
		{
			squares[x][y].clearChildren();
			for (int i = -1; i < 2; ++i)
			{
				newX = squares[x][y].getX() + i;
				for (int j = -1; j < 2; ++j) // for all squares in this 3*3 square
				{
					newY = squares[x][y].getY() + j;
					if (newX > -1 && newX < width && newY > -1 && newY < height) // be sure not to go outside the limits
					{
						aChild = &(squares[newX][newY]);
						if (aChild->getType() && (newX != x || newY != y))
						{  // only take free squares and not the one we are examining
							squares[x][y].addChild(aChild, squares[x][y].localDistanceTo(aChild));
						}
					}
				}
			}
		}
	}
}

coordinate GridPathFinder::coordinate_from_square_pos(int x, int y) {
	return coordinate(x, y) * square_size + arena_min;
}

tuple<int, int> GridPathFinder::square_pos_from_coordinate(coordinate pos) {
	return tuple<int, int>((pos - arena_min).x / square_size, (pos - arena_min).z / square_size);
}

bool GridPathFinder::find_path(coordinate start_pos, coordinate end_pos, vector<coordinate>& coordinates) {
	PathFinder<Square> p;
	std::vector<Square*> path;

	tuple<int, int> start_square = square_pos_from_coordinate(start_pos);
	tuple<int, int> end_square = square_pos_from_coordinate(end_pos);
			
	p.setStart(squares[get<0>(start_square)][get<1>(start_square)]);
	p.setGoal(squares[get<0>(end_square)][get<1>(end_square)]);

	bool r = p.findPath<AStar>(path);
	if (r) {
		coordinates = get_coordinates_from_square_path(path, start_pos, end_pos);
	}
	return r;
}

vector<coordinate> GridPathFinder::get_coordinates_from_square_path(vector<Square*> path, coordinate start_pos, coordinate end_pos) {
	vector<coordinate> path_coords;

	for (int i = 1; i < path.size() - 1; i++) {
		path_coords.push_back(coordinate_from_square_pos(path[i]->getX(), path[i]->getY()));
	}
	path_coords.push_back(end_pos);
	return path_coords;
}

vector<coordinate> GridPathFinder::simplify_path(vector<coordinate> path, double tol) {
	vector<int> simp_vertices;
	simp_vertices.push_back(0);
	simp_vertices.push_back(path.size() - 1);

	bool broken = false;

	for (;;) {
		cout << "Starting simplify iteration" << endl;
		for (int j = 0; j < simp_vertices.size() - 1; j++) {
			coordinate simp_vector = (path[simp_vertices[j + 1]] - path[simp_vertices[j]]).norm();
			vector<double> distances_from_segment;

			for (int i = simp_vertices[j]; i < simp_vertices[j + 1]; i++) {
				coordinate vector_from_segment_start = path[i] - path[simp_vertices[j]];
				double distance_from_line = abs(vector_from_segment_start.z * simp_vector.x - vector_from_segment_start.x * simp_vector.z);
				distances_from_segment.push_back(distance_from_line);
			}

			if (distances_from_segment.size() > 0) {
				auto max_dist = max_element(distances_from_segment.begin(), distances_from_segment.end());
				if (*max_dist > tol) {
					simp_vertices.insert(simp_vertices.begin() + j + 1, max_dist - distances_from_segment.begin() + simp_vertices[j]);
					broken = true;
					break;
				}
			}
		}
		if (broken) {
			broken = false;
		}
		else {
			// we have done a whole run without any distance being greater than the tol, therefore we are done
			break;
		}
	}

	vector<coordinate> simplified_path;
	for (int j = 0; j < simp_vertices.size(); j++) {
		simplified_path.push_back(path[simp_vertices[j]]);
	}

	return simplified_path;
}

void GridPathFinder::delete_point_grid() {
	delete squares;
}

void GridPathFinder::print_grid() {
	cout << "Grid: " << endl;
	for (int x = width-1; x >= 0; x--) {
		for (int y = 0; y < width; y++) {
			cout << (squares[x][y].getType()) ? " " : "X";
		}
		cout << endl;
	}
}

/*
int main(int argc, char** argv)
{
	// Create the PathFinding stuff and the SFML image to load the image
	coordinate arena_max(1.2, 1.2);
	coordinate arena_min(-1.2, -1.2);
	vector<coordinate> blocks = { coordinate(0.5, 0), coordinate(0.7, -0.2)};
	vector<coordinate> path;

	GridPathFinder finder = GridPathFinder(arena_max, arena_min, 0.05);
	finder.add_circles_at_blocks(blocks, 0.15);

	if (finder.find_path(coordinate(0.02, 0.045), coordinate(1, 0), path)) {
		cout << "path found" << endl;
		for (int i = 0; i < path.size(); i++) {
			cout << path[i] << endl;
		}
	}

	finder.delete_point_grid();

	return 0;
}
*/