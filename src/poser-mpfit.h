#pragma once

#include <Eigen/Eigen/Dense>

#include "utils.h"
#include <iostream>
#include <iomanip>
#include "lighthouse-ootx.h"
#include "objects.h"
#include <math.h>
#include <mpfit/mpfit.h>
#include "reprojection-generated.h"
#include "redist/linmath.h"

PoseData CalculatePoseMPFITSingleLH(int lh, LinmathPose initial_pose);
PoseData CalculatePoseMPFITMultiLH(int lh, LinmathPose initial_pose);