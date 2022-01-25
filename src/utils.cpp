#include "utils.h"

int config_angles_to_use = 1;
bool config_apply_lh_calib = false;

Timestamp TimestampDifference(Timestamp most_recent, Timestamp least_recent)
{
    Timestamp diff = 0;
    if (most_recent > least_recent)
    {
        diff = most_recent - least_recent;
    }
    else
    {
        diff = least_recent - most_recent;
    }

    if (diff > 0xFFFFFFFF / 2)
        return 0xFFFFFFFF - diff;
    return diff;
}

bool compareChar(char &c1, char &c2)
{
    if (c1 == c2)
        return true;
    else if (std::toupper(c1) == std::toupper(c2))
        return true;
    return false;
}

bool CaseInSensStringCompare(std::string &str1, std::string &str2)
{
    return ((str1.size() == str2.size()) &&
            std::equal(str1.begin(), str1.end(), str2.begin(), &compareChar));
}

double calculateAzimuth(double angle1, double angle2)
{
    double azimuth = ((angle1 + angle2) / 2);
    return azimuth;
}

double calculateElevation(double angle1, double angle2)
{
    double p = M_PI / 6;
    double tant = tanf(p);
    double omega = (angle2 - angle1);
    double elevation = atan(sin(omega / 2.0) / tan(p));
    return elevation;
}

static inline float clip1(float a)
{
    if (a < -1.0f)
    {
        return -1.0f;
    }

    if (a > 1.0f)
    {
        return 1.0f;
    }

    return a;
}

float lighthouseCalibrationMeasurementModelLh2(const float x, const float y, const float z, const float t, int lh_ch, int plane)
{
    const float ax = atan2f(y, x);
    // const float ay = atan2f(z, x);
    const float r = sqrt(x * x + y * y);

    const float base = ax + asinf(clip1(z * tanf(t - lighthouses[lh_ch].ootx_handler.fcal_tilt[plane]) / r));
    const float compGib = -lighthouses[lh_ch].ootx_handler.fcal_gibmag[plane] * cosf(ax + lighthouses[lh_ch].ootx_handler.fcal_gibphase[plane]);
    // TODO krri Figure out how to use curve and ogee calibration parameters

    return base - (lighthouses[lh_ch].ootx_handler.fcal_phase[plane] + compGib);
}

static void idealToDistortedV2(const float *ideal, float *distorted, int lh_ch)
{
    const float t30 = LINMATHPI / 6.0f;
    const float tan30 = 0.5773502691896258; // const float tan30 = tanf(t30);

    const float a1 = ideal[0];
    const float a2 = ideal[1];

    const float x = 1.0f;
    const float y = tanf((a2 + a1) / 2.0f);
    const float z = sinf(a2 - a1) / (tan30 * (cosf(a2) + cosf(a1)));

    distorted[0] = lighthouseCalibrationMeasurementModelLh2(x, y, z, -t30, lh_ch, 0);
    distorted[1] = lighthouseCalibrationMeasurementModelLh2(x, y, z, t30, lh_ch, 1);
}

void lighthouseCalibrationApply(const float *rawAngles, float *correctedAngles, int lh_ch)
{
    const double max_delta = 0.0005;

    // Use distorted angle as a starting point
    float *estmatedAngles = correctedAngles;
    estmatedAngles[0] = rawAngles[0];
    estmatedAngles[1] = rawAngles[1];

    for (int i = 0; i < 5; i++)
    {
        float currentDistortedAngles[2];
        idealToDistortedV2(estmatedAngles, currentDistortedAngles, lh_ch);

        const float delta0 = rawAngles[0] - currentDistortedAngles[0];
        const float delta1 = rawAngles[1] - currentDistortedAngles[1];

        estmatedAngles[0] = estmatedAngles[0] + delta0;
        estmatedAngles[1] = estmatedAngles[1] + delta1;

        if (fabs((double)delta0) < max_delta && fabs((double)delta1) < max_delta)
        {
            break;
        }
    }
}

void pulseProcessorV2ConvertToV1Angles(const float v2Angle1, const float v2Angle2, float *v1Angles)
{
    const float tant = 0.5773502691896258f; // tan(pi / 6)

    v1Angles[0] = (v2Angle1 + v2Angle2) / 2.0f;
    v1Angles[1] = atan2f(sinf(v2Angle2 - v2Angle1), (tant * (cosf(v2Angle1) + cosf(v2Angle2))));
}

void InitializeDevices(json trackerconfig, bool init_lh_later)
{
    if (!init_lh_later)
    {
        InitializeLighthouses();
    }
    InitializeTracker(trackerconfig);
}

void InitializeLHFromConfig(json ootx_config)
{

    for (int ch = 0; ch < 16; ch++)
    {
        lighthouses[ch].ootx_handler.fillOOTXFromJSON(ootx_config, ch);

        if (lighthouses[ch].ootx_handler.isOOTXInfoAvailable(ch))
        {
            lighthouses[ch].pose_in_univ = lighthouses[ch].ootx_handler.lh_pose;
            if (lighthouses[ch].ootx_handler.isPoseSet)
            {
                is_universe_known = true;
            }
            lighthouses_in_universe++;
        }
    }
}
void InitializeLighthouses()
{

    for (int i = 0; i < NUM_LH_CHANNELS; i++)
    {
        lighthouses[i].ootx_handler.isOOTXInfoAvailable(i);
        if (lighthouses[i].ootx_handler.isOOTXInfoAvailable(i))
        {
            lighthouses[i].pose_in_univ = lighthouses[i].ootx_handler.lh_pose;
            if (lighthouses[i].ootx_handler.isPoseSet)
            {
                is_universe_known = true;
            }
            lighthouses_in_universe++;
        }
    }
}

void InitializeTracker(json trackerconfig)
{

    cout << "tracker json: " << trackerconfig.dump() << endl;

    tracker->head_pose_vive.position[0] = trackerconfig.at("head").at("position")[0];
    tracker->head_pose_vive.position[1] = trackerconfig.at("head").at("position")[1];
    tracker->head_pose_vive.position[2] = trackerconfig.at("head").at("position")[2];

    tracker->head_pose_vive.plus_x[0] = trackerconfig.at("head").at("plus_x")[0];
    tracker->head_pose_vive.plus_x[1] = trackerconfig.at("head").at("plus_x")[1];
    tracker->head_pose_vive.plus_x[2] = trackerconfig.at("head").at("plus_x")[2];
    tracker->head_pose_vive.plus_z[0] = trackerconfig.at("head").at("plus_z")[0];
    tracker->head_pose_vive.plus_z[1] = trackerconfig.at("head").at("plus_z")[1];
    tracker->head_pose_vive.plus_z[2] = trackerconfig.at("head").at("plus_z")[2];

    tracker->imu_pose_vive.position[0] = trackerconfig.at("imu").at("position")[0];
    tracker->imu_pose_vive.position[1] = trackerconfig.at("imu").at("position")[1];
    tracker->imu_pose_vive.position[2] = trackerconfig.at("imu").at("position")[2];
    tracker->imu_pose_vive.plus_x[0] = trackerconfig.at("imu").at("plus_x")[0];
    tracker->imu_pose_vive.plus_x[1] = trackerconfig.at("imu").at("plus_x")[1];
    tracker->imu_pose_vive.plus_x[2] = trackerconfig.at("imu").at("plus_x")[2];
    tracker->imu_pose_vive.plus_z[0] = trackerconfig.at("imu").at("plus_z")[0];
    tracker->imu_pose_vive.plus_z[1] = trackerconfig.at("imu").at("plus_z")[1];
    tracker->imu_pose_vive.plus_z[2] = trackerconfig.at("imu").at("plus_z")[2];

    SolveVivePose(&tracker->head_pose, &tracker->head_pose_vive);
    SolveVivePose(&tracker->imu_pose, &tracker->imu_pose_vive);
    LinmathPose trackref2imu = InvertPoseRtn(&tracker->imu_pose);

    // store normal vector and location of sensor in TrackedObject
    for (int sensorCount = 0; sensorCount < trackerconfig.at("lighthouse_config").at("channelMap").size(); sensorCount++)
    {
        tracker->model_points[sensorCount][0] = trackerconfig.at("lighthouse_config").at("modelPoints")[sensorCount][0];
        tracker->model_points[sensorCount][1] = trackerconfig.at("lighthouse_config").at("modelPoints")[sensorCount][1];
        tracker->model_points[sensorCount][2] = trackerconfig.at("lighthouse_config").at("modelPoints")[sensorCount][2];
        tracker->model_normals[sensorCount][0] = trackerconfig.at("lighthouse_config").at("modelNormals")[sensorCount][0];
        tracker->model_normals[sensorCount][1] = trackerconfig.at("lighthouse_config").at("modelNormals")[sensorCount][1];
        tracker->model_normals[sensorCount][2] = trackerconfig.at("lighthouse_config").at("modelNormals")[sensorCount][2];

        // ApplyPoseToPoint(tracker->model_points[sensorCount], &trackref2imu, tracker->model_points[sensorCount]);

        if (false)
        {
            tracker->model_points[sensorCount][0] = tracker->model_points[sensorCount][0] - tracker->head_pose_vive.position[0];
            tracker->model_points[sensorCount][1] = tracker->model_points[sensorCount][1] - tracker->head_pose_vive.position[1];
            tracker->model_points[sensorCount][2] = tracker->model_points[sensorCount][2] - tracker->head_pose_vive.position[2];
        }
    }
}

Timestamp GetChannelPeriod(int channel)
{
    Timestamp ch_period = double(
        TimestampDifference(lighthouses[channel].current_sync_time, lighthouses[channel].last_sync_time));
    if (ch_period > 800000 && ch_period < 970000)
    { // Min channel period is 887000 and Max 959000 theoretically
        return ch_period;
    }
    return double(channel_periods[channel]);
}

void fillAngles(int channel, int sensor, double angles_arr[MAX_POSSIBLE_SWEEPS_IN_FRAME], int planes_arr[MAX_POSSIBLE_SWEEPS_IN_FRAME])
{
    if (activations->is_timecode_pair_valid[channel][sensor])
    {
        switch (config_angles_to_use)
        {
        case 0: // raw angles - position of rotor at sweep
            activations->angles[channel][sensor][0] = angles_arr[0];
            activations->angles[channel][sensor][1] = angles_arr[1];
            break;
        case 1: // bitcraze c++
        {
            float firstBeam = ((activations->timecode[channel][sensor][0] - lighthouses[channel].last_sync_time) * 2 * M_PI / double(GetChannelPeriod(channel))) - M_PI + M_PI / 3.0f;
            float secondBeam = ((activations->timecode[channel][sensor][1] - lighthouses[channel].last_sync_time) * 2 * M_PI / double(GetChannelPeriod(channel))) - M_PI - M_PI / 3.0f;
            float beam_angles[2] = {secondBeam, firstBeam};
            float corrected_angles[2];
            if (config_apply_lh_calib)
            {
                lighthouseCalibrationApply(beam_angles, corrected_angles, channel);
                activations->angles[channel][sensor][0] = corrected_angles[0];
                activations->angles[channel][sensor][1] = corrected_angles[1];
            }
            else
            {
                corrected_angles[0] = beam_angles[0];
                corrected_angles[1] = beam_angles[1];
            }
            float anglesV1[2];
            pulseProcessorV2ConvertToV1Angles(corrected_angles[0], corrected_angles[1], anglesV1);
            activations->angles[channel][sensor][0] = anglesV1[0];
            activations->angles[channel][sensor][1] = anglesV1[1];

            activations->angles_raw[channel][sensor][0] = beam_angles[0];
            activations->angles_raw[channel][sensor][1] = beam_angles[1];

            break;
        }
        case 2: // bitcrase python
        {
            float anglesAE[2];
            anglesAE[0] = calculateAzimuth(angles_arr[0], angles_arr[1]);
            anglesAE[1] = calculateElevation(angles_arr[0], angles_arr[1]);
            activations->angles[channel][sensor][0] = anglesAE[0];
            activations->angles[channel][sensor][1] = anglesAE[1];

            activations->angles_raw[channel][sensor][0] = angles_arr[0] - M_PI + M_PI / 3.0f;
            activations->angles_raw[channel][sensor][1] = angles_arr[1] - M_PI - M_PI / 3.0f;

            // printf("s: %d ang0: %0.5f ang1: %0.5f\n", sensor, activations->angles_raw[channel][sensor][0], activations->angles_raw[channel][sensor][0]);
            break;
        }

        default:
            break;
        }
    }
}

void ProcessFrame(int channel)
{
    /** TODO: Possible Improvements
	 * Check Valid Pairs if they lye at almost equal distance from sync to remove reflections even more
	 * On Corners of the lighthouse, angles not separately correctly as compared to lh_console. 
	 * i.e. when angle is less than 180 degree currenct code gives it plane 1 on edge
	 * when other plane 0 angle is around 30. But lh console separates them
	 * */

    // cout << "last sync time: " << lighthouses[channel].last_sync_time << endl;

    if (lighthouses[channel].last_sync_time > 0 && lighthouses[channel].ootx_handler.isOOTXavailable)
    {
        int sensor_activations = 0;
        for (int sensor = 0; sensor < NUM_TRACKER_SENSORS; sensor++)
        {
            double angles_arr[MAX_POSSIBLE_SWEEPS_IN_FRAME] = {0};
            int planes_arr[MAX_POSSIBLE_SWEEPS_IN_FRAME] = {0};
            int num_p0_angles = 0;
            int num_p1_angles = 0;
            for (int i = 0; i < MAX_POSSIBLE_SWEEPS_IN_FRAME; i++)
            {

                if (lighthouses[channel].light_frame.timecode[sensor][i] > lighthouses[channel].last_sync_time)
                {
                    Timestamp lh_period = GetChannelPeriod(channel);
                    Timestamp plane_threshold = lighthouses[channel].last_sync_time + lh_period / 2.0;

                    double angle = double(lighthouses[channel].light_frame.timecode[sensor][i] - lighthouses[channel].last_sync_time) / double(lh_period) * 2 * M_PI;
                    // cout << "sensor: " << sensor << " timecode: " << (lighthouses[channel].light_frame.timecode[sensor][i] - lighthouses[channel].last_sync_time) << " i: " << i << endl;
                    angles_arr[i] = angle;
                    int plane = -1;
                    if (lighthouses[channel].light_frame.timecode[sensor][i] < plane_threshold)
                    {
                        // plane = 0;
                        planes_arr[i] = 0;
                        num_p0_angles++;
                    }
                    else
                    {
                        // plane = 1;
                        planes_arr[i] = 1;
                        num_p1_angles++;
                    }
                }
                else
                {
                    lighthouses[channel].light_frame.timecode[sensor][i] = 0; //reset timecode if no hits in current frame
                }
            }
            if (num_p1_angles > 1 || num_p0_angles > 1 || lighthouses[channel].light_frame.sweeps_in_cycle[sensor] > 2)
            {
                activations->timecode[channel][sensor][0] = 0;
                activations->timecode[channel][sensor][1] = 0;
                //reject angles if above conditions are true
            }
            else
            {
                if (lighthouses[channel].light_frame.sweeps_in_cycle[sensor] == 2)
                {
                    activations->is_timecode_pair_valid[channel][sensor] = true;
                    sensor_activations++;
                }
                else
                {
                    activations->is_timecode_pair_valid[channel][sensor] = false;
                }
                // fill sensor activations
                activations->timecode[channel][sensor][0] = lighthouses[channel].light_frame.timecode[sensor][0];
                activations->timecode[channel][sensor][1] = lighthouses[channel].light_frame.timecode[sensor][1];

                fillAngles(channel, sensor, angles_arr, planes_arr);
                // std::cout << "sensor activations: " << sensor_activations << endl;
                activations->sensor_activations[channel] = sensor_activations;
            }

            lighthouses[channel].light_frame.sweeps_in_cycle[sensor] = 0;
        }
    }
}

void PrintVectorDouble(const vector<double> &v)
{
    //vector<int> v;
    for (int i = 0; i < v.size(); i++)
    {
        cout << v[i] << " ";
    }
    cout << endl;
}

void PrintVectorFloat(const vector<float> &v)
{
    //vector<int> v;
    for (int i = 0; i < v.size(); i++)
    {
        cout << v[i] << " ";
    }
    cout << endl;
}

void PrintVectorInt(const vector<int> &v)
{
    //vector<int> v;
    for (int i = 0; i < v.size(); i++)
    {
        cout << v[i] << " ";
    }
    cout << endl;
}

static inline void calc_cal_series(FLT s, FLT *m, FLT *a)
{
    const FLT f[6] = {-8.0108022e-06, 0.0028679863, 5.3685255000000001e-06, 0.0076069798000000001};

    *m = f[0], *a = 0;
    for (int i = 1; i < 6; i++)
    {
        *a = *a * s + *m;
        *m = *m * s + f[i];
    }
}

FLT SurviveReprojectAxisGen2(int lh, FLT X, FLT Y, FLT Z, bool axis)
{
    const FLT phase = lighthouses[lh].ootx_handler.fcal_phase[axis];
    const FLT curve = lighthouses[lh].ootx_handler.fcal_curve[axis];
    const FLT tilt = lighthouses[lh].ootx_handler.fcal_tilt[axis];
    const FLT gibPhase = lighthouses[lh].ootx_handler.fcal_gibphase[axis];
    const FLT gibMag = lighthouses[lh].ootx_handler.fcal_gibmag[axis];
    const FLT ogeePhase = lighthouses[lh].ootx_handler.fcal_ogeephase[axis];
    const FLT ogeeMag = lighthouses[lh].ootx_handler.fcal_ogeemag[axis];

    FLT B = atan2(Z, X);

    FLT Ydeg = tilt + (axis ? -1 : 1) * LINMATHPI / 6.;
    FLT tanA = FLT_TAN(Ydeg);
    FLT normXZ = sqrt(X * X + Z * Z);

    FLT asinArg = linmath_enforce_range(tanA * Y / normXZ, -1, 1);

    FLT sinYdeg = FLT_SIN(Ydeg);
    FLT cosYdeg = FLT_COS(Ydeg);

    FLT sinPart = FLT_SIN(B - FLT_ASIN(asinArg) + ogeePhase) * ogeeMag;

    FLT normXYZ = FLT_SQRT(X * X + Y * Y + Z * Z);

    FLT modAsinArg = linmath_enforce_range(Y / normXYZ / cosYdeg, -1, 1);

    FLT asinOut = FLT_ASIN(modAsinArg);

    FLT mod, acc;
    calc_cal_series(asinOut, &mod, &acc);

    FLT BcalCurved = sinPart + curve;
    FLT asinArg2 = linmath_enforce_range(asinArg + mod * BcalCurved / (cosYdeg - acc * BcalCurved * sinYdeg), -1, 1);

    FLT asinOut2 = asin(asinArg2);
    FLT sinOut2 = sin(B - asinOut2 + gibPhase);

    FLT rtn = B - asinOut2 + sinOut2 * gibMag - phase - PI / 2;

    // printf("axis %d Bcc %0.4f  B %0.4f | asinout2 %0.4f | sinout2 %0.4f | Sgibmag %0.4f | phase %0.4f\n", axis, BcalCurved, B, asinOut2, sinOut2, sinOut2 * gibMag, phase);
    assert(!isnan(rtn));
    return rtn;
}

int SolveVivePose(LinmathPose *pose, const vive_pose_t *vpose)
{
    if (vpose->plus_x[0] == 0.0 && vpose->plus_x[1] == 0.0 && vpose->plus_x[2] == 0.0)
        return 0;

    if (vpose->plus_z[0] == 0.0 && vpose->plus_z[1] == 0.0 && vpose->plus_z[2] == 0.0)
        return 0;

    copy3d(pose->Pos, vpose->position);

    LinmathVec3d plus_y;
    cross3d(plus_y, vpose->plus_z, vpose->plus_x);
    FLT m[] = {vpose->plus_x[0], plus_y[0], vpose->plus_z[0], vpose->plus_x[1], plus_y[1],
               vpose->plus_z[1], vpose->plus_x[2], plus_y[2], vpose->plus_z[2]};

    quatfrommatrix33(pose->Rot, m);

    return 1;
}

float GetDistanceFromLH(LinmathPose *pos)
{
    return sqrt(pow(pos->Pos[0], 2) + pow(pos->Pos[1], 2) + pow(pos->Pos[2], 2));
}