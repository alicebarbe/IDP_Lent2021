/*
// File:          test_controller_cpp.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Emitter.hpp>
#include <webots/Receiver.hpp>

#define RPM_TO_RAD_S 0.1047

// All the webots classes are defined in the "webots" namespace
using namespace webots;

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char** argv) {
  // create the Robot instance.
  Robot* robot = new Robot();

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);

  Motor* leftMotor = robot->getMotor("wheel1");
  Motor* rightMotor = robot->getMotor("wheel2");
  // set the target position of the motors
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  leftMotor->setAvailableTorque(0.4);
  rightMotor->setAvailableTorque(0.4);

  leftMotor->setVelocity(35 * RPM_TO_RAD_S);
  rightMotor->setVelocity(35 * RPM_TO_RAD_S);

  DistanceSensor* leftDistanceSensor = robot->getDistanceSensor("ds_right");
  leftDistanceSensor->enable(10);
  int LUT_size = leftDistanceSensor->getLookupTableSize();
  const double* LUT = leftDistanceSensor->getLookupTable();

  for (int i = 0; i < LUT_size; i++) {
    std::cout << *(LUT + 3 * i) << "  " << *(LUT + 3 * i + 1) << " " << *(LUT + 3 * i + 2) << std::endl;
  }

  std::cout << leftDistanceSensor->getMaxValue() << std::endl;

  Emitter* transmitter = robot->getEmitter("transmitter");

  char message[12] = "Hello World";

  Receiver* receiver = robot->getReceiver("receiver");
  receiver->enable(10);


  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
    transmitter->send(message, 12);

    int datasize = receiver->getDataSize();
    char* data = (char*)receiver->getData();

    for (int i = 0; i < datasize; i++) {
      std::cout << *(data + i) << std::endl;
    }
    std::cout << std::endl;

    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();

    // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
     /// std::cout << leftDistanceSensor->getValue() << std::endl;
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}

*/

// File:          test_controller_cpp.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Emitter.hpp>
#include <webots/Receiver.hpp>

#include <string>

#define RPM_TO_RAD_S 0.1047

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
int main(int argc, char** argv) {
  // create the Robot instance.
  Robot* robot = new Robot();

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);

  // setup motors
  Motor* leftMotor = robot->getMotor("wheel1");
  Motor* rightMotor = robot->getMotor("wheel2");
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);

  // move forward at full speed
  leftMotor->setVelocity(35 * RPM_TO_RAD_S);
  rightMotor->setVelocity(35 * RPM_TO_RAD_S);

  // setup distance sensors
  DistanceSensor* leftDistanceSensor = robot->getDistanceSensor("ds_forward");
  leftDistanceSensor->enable(timeStep);

  // setup emitter and transmitter for communication - note you need two robots to test this
  Emitter* transmitter = robot->getEmitter("transmitter");
  Receiver* receiver = robot->getReceiver("receiver");
  receiver->enable(timeStep);


  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
    // transmit a message
    char message[12] = "Hello World";
    transmitter->send(message, 12);

    
    // if a message is in the receiver buffer, read it and print to console
    while (receiver->getQueueLength() > 0) {
      int datasize = receiver->getDataSize();
      char* data = (char*)receiver->getData();
      receiver->nextPacket(); //reading the packet doesnt pop it from the queue!

      string data_string(data);
      cout << data_string << endl;
    }
    
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();

    // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
     /// std::cout << leftDistanceSensor->getValue() << std::endl;
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}

