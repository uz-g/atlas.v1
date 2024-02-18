#include "main.h"
#include <fstream>
#include "okapi/api.hpp"
#include <iostream>
#include "lemlib/api.hpp"
#pragma

const int LEFT_MTR_B = 1; // |
const int LEFT_MTR_M = 6; // |
const int LEFT_MTR_F = 3; // |
const int RIGHT_MTR_B = 7; // |
const int RIGHT_MTR_M = 8; // |
const int RIGHT_MTR_F = 9; // |
const int INTAKE = 10; // |
const int FLYWHEEL = 2;
const int rightRotationSensor = 18; // |
const int leftRotationSensor = 16; // |
//port 11-13 on the brain is damaged

// distance controller gains
const float dkP = 0.0001;
const float dkI = 0.0000; // i is usually not needed for vex pids
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