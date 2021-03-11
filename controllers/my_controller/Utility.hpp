#pragma once

#include <tuple>
#include <vector>

#include "Coordinate.hpp"
#include "Movement.hpp"

double getCompassBearing(const double* vector);
double getBearing(coordinate vector);
double getBearingDifference(double bearingOne, double bearingTwo);
double constrainBearing(double bearing);
coordinate rotateVector(const coordinate vector, double angle);
double getPIDOutput(double error, PIDGains gains, PIDState state);
double distanceBetweenPoints(coordinate pos1, coordinate pos2);