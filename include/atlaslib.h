#include "main.h"
#include "okapi/api.hpp"

#pragma

const int LEFT_MTR1 = 10;
const int LEFT_MTR2 = 9;
const int LEFT_MTR3 = 8;
const int RIGHT_MTR1 = 1;
const int RIGHT_MTR2 = 2;
const int RIGHT_MTR3 = 3;
const int WINGS = 6;
const int PUNCHER = 5;
const int yRotation = 4;
const int xRotation = 7;

// distance controller gains
const float kPDist = 0.0001;
const float kIDist = 0.0002;
const float kDDist = 0.0010002;

// turn controller gains
const float kPTurn = 0.0001;
const float kITurn = 0.0002;
const float kDTurn = 0.0010002;

// angle controller gains
const float kPAngle = 0.0001;
const float kIAngle = 0.0002;
const float kDAngle = 0.0010002;

const double kMaxVel = 5.6;   // inches per second
const double kMaxAccel = 1.5; // desired inches per second squared acceleration
const double kMaxJerk = .9;   // desired inches per second cubed jerk (rate of change of acceleration)

