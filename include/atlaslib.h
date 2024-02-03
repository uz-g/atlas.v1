#include "main.h"
#include <fstream>
#include "okapi/api.hpp"
#include <iostream>
#include "lemlib/api.hpp"
#pragma

const int LEFT_MTR_B = 18;
const int LEFT_MTR_M = 10;
const int LEFT_MTR_F = 19;
const int RIGHT_MTR_B = 9;
const int RIGHT_MTR_M = 1;
const int RIGHT_MTR_F = 3;
const int FLYWHEEL = 6;
const int INTAKE = 5;
const int rightRotationSensor = 4;
const int leftRotationSensor = 7;

// distance controller gains
const float dkP = 0.0001;
const float dkI = 0.0000; //i is usually not needed for vex pids
const float dkD = 0.0001;

// turn controller gains
const float tkP = 0.0001;
const float tkI = 0.0000;
const float tkD = 0.0001;

// angle controller gains
const float akP = 0.0001;
const float akI = 0.0000;
const float akD = 0.0001;

/*
the robot for atlasV1 is a 6 motor drive, 5.5w intake and 16.5w cata
*/