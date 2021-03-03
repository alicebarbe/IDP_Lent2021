#include <webots/Motor.hpp>
#include <tuple>

void moveToPosition(std::tuple<webots::Motor*, webots::Motor*> motors);
void turnToBearing(std::tuple<webots::Motor*, webots::Motor*> motors,double bearing);