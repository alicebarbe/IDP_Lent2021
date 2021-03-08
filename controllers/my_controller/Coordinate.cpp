#include <iostream>
#include <tuple>

#include "coordinate.hpp"

using namespace std;

coordinate coordinate::operator+(const const coordinate& c) {
  coordinate coord;
  coord.x = c.x + this->x;
  coord.z = c.z + this->z;
  return coord;
}
coordinate coordinate::operator-(const const coordinate& c) {
  coordinate coord;
  coord.x = this->x - c.x;
  coord.z = this->z - c.z;
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

coordinate::coordinate() {
  this->x = 0;
  this->z = 0;
}