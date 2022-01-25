#pragma once

#include "utils.h"

#include <iomanip>
#include "objects.h"
#include <math.h>
#include "redist/linmath.h"
#include "poser-p3p-lt-reprojection.h"
#include <boost/histogram.hpp>
#include <boost/histogram/detail/fill.hpp>
#include <boost/histogram/detail/fill_n.hpp>

LinmathPose CalculatePoseP3LTPReproj(int lh);

using namespace boost::histogram;