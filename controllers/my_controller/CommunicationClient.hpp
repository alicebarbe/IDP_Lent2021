#pragma once

#include <webots/Robot.hpp>
#include <webots/GPS.hpp>
#include <webots/Compass.hpp>
#include <webots/Emitter.hpp>
#include <webots/Receiver.hpp>

typedef std::tuple<int, double, double> message;

//char* received_data;

webots::GPS* initGPS(webots::Robot* robot, const char* name);
const double* getlocation(webots::GPS* gps);
webots::Compass* initCompass(webots::Robot* robot, const char* name);
const double* getDirection(webots::Compass* compass);
webots::Emitter* initEmitter(webots::Robot* robot, const char* name);
webots::Receiver* initReceiver(webots::Robot* robot, const char* name);
void emitData(webots::Emitter* emitter, const void* data, int size);
message* receiveData(webots::Receiver* receiver);