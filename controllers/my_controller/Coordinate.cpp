#include <iostream>
#include <tuple>

#include "coordinate.hpp"

using namespace std;

coordinate coordinate::operator+(const coordinate& c) {
  coordinate coord;
  coord.x = c.x + this->x;
  coord.z = c.z + this->z;
  return coord;
}
coordinate coordinate::operator-(const coordinate& c) {
  coordinate coord;
  coord.x = this->x - c.x;
  coord.z = this->z - c.z;
  return coord;
}

coordinate coordinate::operator*(const double& d) {
  coordinate coord;
  coord.x = this->x * d;
  coord.z = this->z * d;
  return coord;
}

ostream& operator<<(ostream& os, const coordinate& c) {
  os << "(" << c.x << ", " << c.z << ")";
  return os;
}

coordinate::coordinate(double x, double z) {
  this->x = x;
  this->z = z;
}

coordinate::coordinate(tuple<double, double> coord) {
  this->x = std::get<0>(coord);
  this->z = std::get<1>(coord);
}

coordinate::coordinate(const double* gpsLoc) {
  this->x = gpsLoc[0];
  this->z = gpsLoc[2];
}

coordinate::coordinate() {
  this->x = 0;
  this->z = 0;
}