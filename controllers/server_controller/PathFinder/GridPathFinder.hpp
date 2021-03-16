#pragma once

#include "PathFinder.h"
#include "AStar.h"
#include "../../my_controller/Coordinate.hpp"

#include <chrono>
#include <iostream>
#include <vector>

class Square : public AStarNode
{
public:
	Square();
	~Square();

	void setType(const bool type);
	bool getType() const;
	float localDistanceTo(AStarNode* node);
	float distanceTo(AStarNode* node) const;

private:
	// To tell wether a pixel is "walkable" or not
	bool m_type;
};

class GridPathFinder
{
	Square** squares;
	coordinate arena_max;
	coordinate arena_min;
	double square_size;

	int width, height;

public:
	GridPathFinder(coordinate a_max, coordinate a_min, double sq_size);
	~GridPathFinder();
	void make_point_grid();
	void update_children();
	void add_circles_at_blocks(std::vector<coordinate> blocks, double radius);

	coordinate coordinate_from_square_pos(int x, int y);
	std::tuple<int, int> square_pos_from_coordinate(coordinate pos);
	bool find_path(coordinate start_pos, coordinate end_pos, std::vector<coordinate>& coordinates);
	std::vector<coordinate> get_coordinates_from_square_path(std::vector<Square*> path, coordinate start_pos, coordinate end_pos);
	std::vector<coordinate> simplify_path(std::vector<coordinate> path, double tol);
	void add_circle_at_robot(coordinate robot_position, double radius);
	void delete_point_grid();

	void print_grid(coordinate start, coordinate end);
};