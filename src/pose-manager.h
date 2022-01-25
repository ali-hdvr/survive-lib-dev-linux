#pragma once

#include "utils.h"

#include <iomanip>
// #include "opencv_pose_calc.h"

#include "objects.h"
#include "definitions.h"
#include <math.h>
#include "poser-p3p-lt-reprojection-wrapper.h"
#include "poser-mpfit.h"
#include "io-handler.h"

using namespace std;

PoseData CalculatePose(int lh);