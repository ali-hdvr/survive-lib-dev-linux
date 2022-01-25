#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include <stdint.h>
#include "lighthouse-ootx.h"
#include "spree/interface.h"

#include <redist/linmath.h>
#include <string>
#include "json/json.hpp"
#include <iostream>
#include <iomanip>
#include <sstream>
#include "spree/poser.h"

#define TRACKER_CLK_FREQ ((float)48000000.0)
#define NUM_LH_CHANNELS 16
#define NUM_TRACKER_SENSORS 22

#define MAX_POSSIBLE_SWEEPS_IN_FRAME 10 //To store sweeps from reflections as well. Max Possible sweeps per sensor

using namespace std;
static double freq_per_channel[NUM_LH_CHANNELS] = {
    50.0521,
    50.1567,
    50.3673,
    50.5796,
    50.6864,
    50.9014,
    51.0096,
    51.1182,
    51.2273,
    51.6685,
    52.2307,
    52.6894,
    52.9217,
    53.2741,
    53.7514,
    54.1150,
};

static double channel_periods[NUM_LH_CHANNELS] = {
    959000,
    957000,
    953000,
    949000,
    947000,
    943000,
    941000,
    939000,
    937000,
    929000,
    919000,
    911000,
    907000,
    901000,
    893000,
    887000};

struct LightFrame
{

    int sweeps_in_cycle[NUM_TRACKER_SENSORS] = {0};
    Timestamp timecode[NUM_TRACKER_SENSORS][MAX_POSSIBLE_SWEEPS_IN_FRAME] = {0}; // Timecode per axis in ticks : only 2 sweeps should be arriving but storing 6 in case there are reflections
};

struct Lighthouse
{
    int id;         // id obtained from ootx
    int index = -1; // local id assigned to LH e.g. 0, 1, 2, 3
    int channel;
    int position_set = 0;
    LinmathPose pose_in_univ = {{0}, {0}};
    Timestamp last_sync_time = 0;
    Timestamp current_sync_time = 0;
    LighthouseOOTX ootx_handler;
    LightFrame light_frame;
    float lm_error = 0;
};

typedef struct
{
    FLT position[3];
    FLT plus_x[3];
    FLT plus_z[3];
} vive_pose_t;

struct Tracker
{
    int num_sensors = NUM_TRACKER_SENSORS;
    string serial;
    double sensor_scale = 1.0034; // different for tracker 2.0 and 3.0. this one is for 3.0 https://github.com/cntools/libsurvive/blob/7ad1f8f2723c03754639f9fb20c913c8c8cfaf39/src/survive_default_devices.c#L282 and /steamapps/common/SteamVR/drivers/lighthouse/resources/
    LinmathVec3d model_normals[NUM_TRACKER_SENSORS];
    LinmathVec3d model_points[NUM_TRACKER_SENSORS];
    vive_pose_t head_pose_vive;
    LinmathPose head_pose = {{0}, {0}};
    vive_pose_t imu_pose_vive;
    LinmathPose imu_pose = {{0}, {0}};
    LinmathPose pose_in_universe = {{0}, {0}};
    LinmathPose pose_imu2head = {{0}, {0}};
    LinmathPose pose_tracker2head = {{0}, {0}};
    double lm_error;
};
enum Phase
{
    Phase_Calibration,
    Phase_PoseEstimation,
    Phase_Tracking
};

/**
 * This struct encodes what the last effective angles seen on a sensor were, and when they occured.
 */
struct SensorActivations
{

    // Valid for gen2; somewhat different meaning though -- refers to angle of the rotor when the sweep happened.
    FLT angles[NUM_LH_CHANNELS][NUM_TRACKER_SENSORS][2] = {0};         // 2 Axes (Angles in LH space)
    Timestamp timecode[NUM_LH_CHANNELS][NUM_TRACKER_SENSORS][2] = {0}; // Timecode per axis in ticks : only 2 sweeps should be arriving but storing 6 in case there are reflections
    bool is_timecode_pair_valid[NUM_LH_CHANNELS][NUM_TRACKER_SENSORS] = {0};
    int sensor_activations[NUM_LH_CHANNELS]; // total sensor pairs active in current scene
    Timestamp last_imu;
    int imu_obs_count = 0; // number of imu observations;
    Timestamp last_light;
    Timestamp last_movement; // Tracks the timecode of the last IMU packet which saw movement.
    Timestamp last_pose_ts[NUM_LH_CHANNELS] = {0};
    LinmathPose pose_prev[NUM_LH_CHANNELS] = {{0}, {0}};
    FLT accel[3];
    FLT gyro[3];
    FLT angles_raw[NUM_LH_CHANNELS][NUM_TRACKER_SENSORS][2] = {0};

    FLT angles_center[NUM_LH_CHANNELS][2] = {0};
    int angles_center_cnt[NUM_LH_CHANNELS][2] = {0};
};

struct PoseData
{
    bool is_updated = false;
    PoseMethod pose_method;
    LinmathPose pose = {{0}, {0}};
};

#endif