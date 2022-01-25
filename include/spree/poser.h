#ifndef ADD_HPP
#define ADD_HPP

#include "spree/interface.h"
#include "json/json.hpp"

typedef double SpreePoint3d[3];

typedef double SpreeQuat[4]; // This is the [wxyz] quaternion, in wxyz format.

typedef struct SpreePose
{
    SpreePoint3d Pos;
    SpreeQuat Rot;
} SpreePose;

enum PoseMethod
{
    P3P,
    MPFS,
    MPFM
};

typedef struct SpreePoseData
{
    SpreePose pose = {{0}, {0}};
    PoseMethod pose_method;
    bool is_updated = false;
} SpreePoseData;

namespace Poser
{
    SpreePoseData Sync(SYNC sync);
    void Sweep(SWEEP sweep);
    void Inertial(IMU imu);
    void SetPoseOnSync(bool pose_every_sync);
    void UpdateTrackerConfig(nlohmann::json config_t, bool init_lh_later = false);

    void UpdateLighthouseConfig(nlohmann::json config_l);
}
#endif