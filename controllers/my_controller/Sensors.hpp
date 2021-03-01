
#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/LightSensor.hpp>

using namespace webots;
using namespace std;

DistanceSensor* initDistanceSensor(Robot* robot, const char* name);
int getDistanceMeasurement(DistanceSensor* ds);