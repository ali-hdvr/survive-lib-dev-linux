#pragma once

#include "utils.h"
#include <iostream>
#include <iomanip>
#include "lighthouse-ootx.h"

#include "objects.h"

#include <math.h>

#include "spree/poser.h"
#include "pose-manager.h"
#include "io-handler.h"

using namespace std;

void ControllerIMU(IMU imu);
SpreePoseData ControllerSync(SYNC sync);
void ControllerSweep(SWEEP sweep);

void ControllerSetPoseEverySync(bool pose_every_sync = false);
