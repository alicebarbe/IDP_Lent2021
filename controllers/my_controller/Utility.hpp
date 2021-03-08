#pragma once

#include <tuple>
#include <vector>

double getCompassBearing(const double* vector);
double getBearing(std::tuple<double, double> vector);
double getBearingDifference(double bearingOne, double bearingTwo);
std::tuple<double, double> rotateVector(const std::tuple<double, double> vector, double angle);
double getWallDistance(const double* robot_pos, double angle, const std::tuple<double, double> sensorDisplacement);