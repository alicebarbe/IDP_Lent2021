#pragma once

#include <tuple>
#include <vector>

#include "Coordinate.hpp"

double getCompassBearing(const double* vector);
double getBearing(coordinate vector);
double getBearingDifference(double bearingOne, double bearingTwo);
double constrainBearing(double bearing);
coordinate rotateVector(const coordinate vector, double angle);
double getWallDistance(const coordinate robotPos, double angle);