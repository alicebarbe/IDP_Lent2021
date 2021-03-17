#pragma once

#define _USE_MATH_DEFINES
#include <math.h>
#include <cmath>
#include <tuple>

#include "Coordinate.hpp"

#define RED_ROBOT 2
#define GREEN_ROBOT 1

inline const double RPM_TO_RAD_S = M_PI / 30.0;
inline const double DEG_TO_RAD  = M_PI / 180.0;
inline const double RAD_TO_DEG  = 180.0 / M_PI;

inline const double MOTOR_MAX_SPEED  = 35 * RPM_TO_RAD_S;  // rads-1
inline const double SERVO_MAX_SPEED  = 58.9 * RPM_TO_RAD_S;  // rads-1
inline const double SERVO_TRAVEL  = 90;  // degrees

inline const double ULTRASOUND_BEAM_ANGLE = 0;  //degrees
inline const double ULTRASOUND_MIN_DISTANCE = 0.03; //m

inline const double BLOCK_SIZE = 0.05;  // m

inline const double ARENA_X_MAX = 1.2;  // m
inline const double ARENA_X_MIN = -1.2;  // m
inline const double ARENA_Z_MAX = 1.2;  // m
inline const double ARENA_Z_MIN = -1.2;  // m

inline const int COMPARATOR_REF_RED = 420;  // 10 bit value
inline const int COMPARATOR_REF_GREEN = 420;  // 10 bit value

inline const coordinate frontOfRobotDisplacement(0.082, 0.015); // dont use this to tune movement - this must equal the displacement of a block in the grabber arm for positioning
inline const coordinate distanceSensorDisplacement(0.082, 0.0);
inline const coordinate rightMostPointDispacement(0.0, 0.123);  //A vector out from the GPS to the outer edge of the wheel
inline const coordinate collisionCircleCentre(-0.035, 0);  //A vector to the midpoint along the lenght of the robot for collision avoidance
inline const double forwardMostRobotPointDist = 0.13;  // forward distance of the tip of the fixed arm of the robot from the GPS
inline const double collisionCircleRadius = 0.25;  // the radius of circle, centred at collisionCircleCenter, to use for collision avoidance
inline const double eatBlockDistance = 0.16;
inline const double otherRobotProximityThresh = 0.75; // (m) The distance from the other robot to trigger a collision event (if the other robot is in this robots path)
inline const double robotHalfWidthClearance = 0.17;  // the clearance to use for collision detection
inline const double closestDistanceBlockFromWall = 0.05;  // we dont attempt to go closer to the wall than this
inline const double closestPathfindDistanceToWall = 0.15; // we dont allow the A* algorithm to go closer than this to the wall
inline const double approachBlockPerpendicularFromWallDistance = 0.15;  //(m) approach blocks closer to the wall than this perpendicular to the wall

