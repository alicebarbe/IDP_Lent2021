
// File:          my_controller.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <math.h>
#include <stdlib.h>
#include <iostream>
#include <tuple>
#include <vector>

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>

#include "CommunicationClient.hpp"
#include "Sensors.hpp"
#include "Movement.hpp"
#include "Motors.hpp"
#include "Servos.hpp"

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;

//typedef tuple<int, double, double> message;

bool scan_for_blocks();
void drive_to_block(void);

bool BLOCK_DETECTED = 0;

double val1 = 0;
static double oldval1 = 0;
double blockdistance = 0;
Robot *robot = new Robot();
tuple<Motor*, Motor*> motors = initMotors(robot, "wheel1", "wheel2");
DistanceSensor* ds1 = initDistanceSensor(robot, "ds_right");
Motor* gripperservo = initServo(robot, "gripper motor");

GPS* gps = initGPS(robot, "gps");
Compass* compass = initCompass(robot, "compass");
tuple<LightSensor*, LightSensor*> colour_sensor = initLightSensor(robot, "light_sensor_red", "light_sensor_green");
Receiver* receiver = initReceiver(robot, "receiver");
Emitter* emitter = initEmitter(robot, "emitter");

/*
vector<tuple<double, double> > target_points = { tuple<double, double>(-0.4, 0.8),
                                                tuple<double, double>(-0.4, 0.0),
                                                tuple<double, double>(0.4, 0.0),
                                                tuple<double, double>(0.4, 0.8) };
*/

int main(int argc, char** argv) {
    // get the time step of the current world.
    int timeStep = (int)robot->getBasicTimeStep();

    // - perform simulation steps until Webots is stopping the controller
    while (robot->step(timeStep) != -1) {
        // Read the sensors:
        oldval1 = val1;
        val1 = getDistanceMeasurement(ds1);
        /*
        const double* bearing = getDirection(compass);
        const double* pos = getlocation(gps);

        tuple<double, double> position(pos[0], pos[2]);
        tuple<double, double> motor_speeds = moveToPosition(position, bearing);
        setMotorVelocity(motors, motor_speeds);
        if (hasReachedPosition()) {
          cout << "Arrived!" << endl;
          updateTargetPosition(target_points[i]);
          i = (i+1) % 4;
        }
        */

        //cout << getLightMeasurement(ls1) << endl;
        //cout << getlocation(gps)[0] << getlocation(gps)[1] << getlocation(gps)[2] << endl;
        //cout << "x:" << getDirection(compass)[0] << "y:" << getDirection(compass)[1] << "z:" <<getDirection(compass)[2] << endl;
        // Enter here functions to read sensor data, like:

        
        if(!BLOCK_DETECTED){
          scan_for_blocks();}
        else {
            static char i = 0;
            if (i == 0) {
                message yessir{};
                yessir = { 14 ,1.1,2.4 };
                const void* alpha = &yessir;
               
                emitData(emitter, alpha, 17);//"block found", 12);
                i++;
                }
            drive_to_block();
             }
        //char* received_data = receiveData(receiver);
       /* if(received_data){
            string data_string(received_data);
            cout << data_string << endl;
        }*/

      };
      

        delete robot;
        return 0;
    }



bool scan_for_blocks(){ 
  static char i = 0;
  static bool SENSORHEATUP = 0;
     
  
  if (i<3){
    SENSORHEATUP = 1;
    i+=1;}
  else{SENSORHEATUP = 0;}
       

  if(std::abs(val1-oldval1) > 0.05 && !SENSORHEATUP)
  {
    setMotorVelocity(motors, tuple<double, double>(0.0, -0.0));
    BLOCK_DETECTED = 1;
    blockdistance = val1;
  }
       
  else if(!BLOCK_DETECTED){
    setMotorVelocity(motors, tuple<double, double>(0.5, -0.5));
  }

  oldval1 = val1;
  return(BLOCK_DETECTED);
}
     
void drive_to_block(){
  if(val1-blockdistance > 0.1){
    setMotorVelocity(motors, tuple<double, double>(0.5, -0.5));
  }
  else if(blockdistance > 0.05){
    setMotorVelocity(motors, tuple<double, double>(6.0, 6.0));
    blockdistance = val1;
  }
  else {
    setMotorVelocity(motors, tuple<double, double>(0.0, 0.0));
    openGripper(gripperservo);
    blockdistance = val1;
  }
}

