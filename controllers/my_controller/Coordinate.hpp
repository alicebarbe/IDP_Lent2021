#pragma once
#include <tuple>
#include <iostream>

class coordinate {
public:
  double x;
  double z;
  coordinate(double x, double z);
  coordinate(std::tuple<double, double>);
  coordinate(const double* gpsLoc);
  coordinate();

  coordinate operator-(const coordinate& c);
  coordinate operator+(const coordinate& c);
  coordinate operator*(const double& d);
  bool operator==(const coordinate& c);
  friend std::ostream& operator<<(std::ostream& os, const coordinate& c);

  coordinate norm();
};