#pragma once

#include <stdint.h>
#include "redist/linmath.h"
#include "definitions.h"
#include <string>
#include <iostream>
#include <sstream>
#include <iomanip>

using namespace std;

int PrintPoseLibsurviveFormatLM(Timestamp ts, std::string object_name, LinmathPose *pose);