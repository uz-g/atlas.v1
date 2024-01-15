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


//pid stuff
const double kP = 0.1;
const double kI = 0.01;
const double kD = 0.1;