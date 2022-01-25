#pragma once

#include "utils.h"
#include <iostream>
#include <iomanip>
#include "lighthouse-ootx.h"
#include "objects.h"
#include <math.h>

#include "Eigen/Eigen/Dense"
#include "p3p.h"

using namespace std;
using namespace lambdatwist;

Eigen::MatrixXf CalculatePoseP3PLambdaTwistReproj(int lh);