// File:          ServerController.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Emitter.hpp>
#include <webots/Receiver.hpp>

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  Emitter* transmitter_one = robot->getEmitter("transmitter_one");
  Receiver* receiver_one = robot->getReceiver("receiver_one");
  receiver_one->enable(timeStep);

  Emitter* transmitter_two = robot->getEmitter("transmitter_two");
  Receiver* receiver_two = robot->getReceiver("receiver_two");
  receiver_two->enable(timeStep);

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();

    while (receiver_one->getQueueLength() > 0) {
      int datasize = receiver_one->getDataSize();
      char* data = (char*)receiver_one->getData();
      receiver_one->nextPacket(); //reading the packet doesnt pop it from the queue!

      string data_string(data);
      cout << "(Server) Robot one says:" << data_string << endl;
    }

    while (receiver_one->getQueueLength() > 0) {
      int datasize = receiver_two->getDataSize();
      char* data = (char*)receiver_two->getData();
      receiver_two->nextPacket(); //reading the packet doesnt pop it from the queue!

      string data_string(data);
      cout << "(Server) Robot two says:" << data_string << endl;
    }

    // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
