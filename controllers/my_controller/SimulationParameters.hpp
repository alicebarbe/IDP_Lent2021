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

inline const int COMPARATOR_REF_RED = 500;  // 10 bit value
inline const int COMPARATOR_REF_GREEN = 500;  // 10 bit value

inline const coordinate frontOfRobotDisplacement(0.082, 0.00);
inline const coordinate distanceSensorDisplacement(0.082, 0.0);
inline const double eatBlockDistance = 0.15;

