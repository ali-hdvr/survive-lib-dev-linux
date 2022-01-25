#include "spree/poser.h"
#include <iostream>
#include "controller.h"
#include "utils.h"

using namespace std;

void Poser::Sweep(SWEEP sweep)
{
    // cout << "Sweep: time: " << sweep.time << " ch: " << (unsigned)sweep.channel << " sensor: " << (unsigned)sweep.sensor << endl;
    ControllerSweep(sweep);
}

SpreePoseData Poser::Sync(SYNC sync)
{
    // cout << "Sync: time: " << sync.time << " ch: " << (unsigned)sync.channel << " ootx: " << sync.ootx << endl;
    return ControllerSync(sync);
}

void Poser::Inertial(IMU imu)
{
    // cout << "Sync: time: " << sync.time << " ch: " << (unsigned)sync.channel << " ootx: " << sync.ootx << endl;
    ControllerIMU(imu);
}

void Poser::UpdateTrackerConfig(nlohmann::json config_t, bool init_lh_later)
{
    InitializeDevices(config_t, init_lh_later);
}

void Poser::UpdateLighthouseConfig(nlohmann::json config_l)
{
    InitializeLHFromConfig(config_l);
}

void Poser::SetPoseOnSync(bool pose_every_sync)
{
    ControllerSetPoseEverySync(pose_every_sync);
}