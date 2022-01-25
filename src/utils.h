#pragma once

#include <redist/linmath.h>
#include <string>
#include "json/json.hpp"
#include <iostream>
#include <iomanip>
#include <sstream>
#include "definitions.h"
#include "objects.h"

using namespace std;

#define degToRad(angleInDegrees) ((angleInDegrees)*M_PI / 180.0)
#define radToDeg(angleInRadians) ((angleInRadians)*180.0 / M_PI)

typedef nlohmann::json json;

void InitializeTracker(json trackerconfig);
void InitializeDevices(json trackerconfig, bool init_lh_later = false);
void InitializeLHFromConfig(json ootx_config);
void InitializeLighthouses();

Timestamp TimestampDifference(Timestamp most_recent, Timestamp least_recent);

double calculateAzimuth(double angle1, double angle2, double tilt1, double tilt2);

double calculateElevation(double angle1, double angle2, double tilt1, double tilt2);

template <typename T>
std::string to_stringp(const T a_value, const int n = 6)
{
    std::ostringstream out;
    out.precision(n);
    out << setw(10) << std::fixed << a_value;
    return out.str();
}

typedef struct
{
    float phase;
    float tilt;
    float curve;
    float gibmag;
    float gibphase;
    // Lh2 extra params
    float ogeemag;
    float ogeephase;
} lighthouseCalibrationSweep_t;

typedef struct
{
    lighthouseCalibrationSweep_t sweep[2];
    bool valid;
} lighthouseCalibration_t;

void lighthouseCalibrationApply(const lighthouseCalibration_t *calib, const float *rawAngles, float *correctedAngles);
void pulseProcessorV2ConvertToV1Angles(const float v2Angle1, const float v2Angle2, float *v1Angles);
Timestamp GetChannelPeriod(int channel);
void ProcessFrame(int channel);

bool CaseInSensStringCompare(std::string &str1, std::string &str2);

void PrintVectorDouble(const vector<double> &v);
void PrintVectorFloat(const vector<float> &v);
void PrintVectorInt(const vector<int> &v);

FLT SurviveReprojectAxisGen2(int lh, FLT X, FLT Y, FLT Z, bool axis);

int SolveVivePose(LinmathPose *pose, const vive_pose_t *vpose);

float GetDistanceFromLH(LinmathPose *pos);
