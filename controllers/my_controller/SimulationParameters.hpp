#pragma once

#define _USE_MATH_DEFINES
#include <math.h>
#include <cmath>
#include <tuple>

inline const double RPM_TO_RAD_S = M_PI / 30.0;
inline const double DEG_TO_RAD  = M_PI / 180.0;
inline const double RAD_TO_DEG  = 180.0 / M_PI;

inline const double MOTOR_MAX_SPEED  = 35 * RPM_TO_RAD_S;  // rads-1
inline const double SERVO_MAX_SPEED  = 58.9 * RPM_TO_RAD_S;  // rads-1
inline const double SERVO_TRAVEL  = 90;  // degrees

inline const double ULTRASOUND_BEAM_ANGLE = 0;  //degrees

inline const double BLOCK_SIZE = 0.05;  // m

inline const std::tuple<double, double> frontOfRobotDisplacement(0.08, 0.015);
inline const std::tuple<double, double> distanceSensorDisplacement(0.05, 0.0);

