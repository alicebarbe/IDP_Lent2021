#include <iostream>
#include <tuple>

#include "coordinate.hpp"

using namespace std;

coordinate coordinate::operator+(const coordinate& c) {
  coordinate coordinate;
  coordinate.x = c.x + this->x;
  coordinate.z = c.z + this->z;
  return coordinate;
}
coordinate coordinate::operator-(const coordinate& c) {
  coordinate coordinate;
  coordinate.x = this->x - c.x;
  coordinate.z = this->z - c.z;
  return coordinate;
}

coordinate coordinate::operator*(const double& d) {
  coordinate coordinate;
  coordinate.x = this->x * d;
  coordinate.z = this->z * d;
  return coordinate;
}

bool coordinate::operator==(const coordinate& c) {
  return (this->x == c.x) || (this->z == c.z);
}

ostream& operator<<(ostream& os, const coordinate& c) {
  os << "(" << c.x << ", " << c.z << ")";
  return os;
}

coordinate::coordinate(double x, double z) {
  this->x = x;
  this->z = z;
}

coordinate::coordinate(tuple<double, double> coordinate) {
  this->x = std::get<0>(coordinate);
  this->z = std::get<1>(coordinate);
}

coordinate::coordinate(const double* gpsLoc) {
  this->x = gpsLoc[0];
  this->z = gpsLoc[2];
}

coordinate::coordinate() {
  this->x = 0;
  this->z = 0;
}

coordinate coordinate::norm() {
  return (*this) * (1.0 / sqrt(pow(x, 2) + pow(z, 2)));
}