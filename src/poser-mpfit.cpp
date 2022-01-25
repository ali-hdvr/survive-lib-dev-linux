#include "poser-mpfit.h"

bool mpfit_inititialized = false;
mp_config config_single_lh;
mp_result result_single_lh;

mp_config config_multi_lh;
mp_result result_multi_lh;

float sensor_variance = 1;

/*

survive_solver.c line 30
struct mp_result_single_lh_struct result_single_lh = {0};
result_single_lh.resid = alloca(mpctx->measurementsCnt * sizeof(double));
result_single_lh.xerror = alloca(survive_optimizer_get_parameters_count(mpctx) * sizeof(double));
result_single_lh.covar = alloca(survive_optimizer_get_parameters_count(mpctx) *
survive_optimizer_get_parameters_count(mpctx) * sizeof(double));
*/

struct Measurement
{
    int lh_ch = -1;
    int sensor_id = -1;
    int axis = -1;
    FLT pixel_pt = -1;
    LinmathVec3d sensor_point_3D = {0};
};

struct mpfit_private_data
{
    vector<Measurement> meas;
    LinmathAxisAnglePose obj2world;
};

void init_mpfit()
{
    memset(&config_single_lh, 0, sizeof(config_single_lh));
    memset(&result_single_lh, 0, sizeof(result_single_lh));

    config_single_lh.xtol = 0.001;

    memset(&config_multi_lh, 0, sizeof(config_multi_lh));
    memset(&result_multi_lh, 0, sizeof(result_multi_lh));

    config_multi_lh.xtol = 0.001;
}

static int GetExplicitJacobians(int m, int n, FLT *p, FLT **derivs, void *p_data, int meas_idx, LinmathAxisAnglePose *pose, LinmathAxisAnglePose *lh_P_w)
{
    mpfit_private_data *private_data = (mpfit_private_data *)p_data;

    FLT jout[7 * 2] = {0};
    gen_reproject_gen2_jac_obj_p_axis_angle(jout, pose, private_data->meas[meas_idx].sensor_point_3D, lh_P_w, private_data->meas[meas_idx].lh_ch);

    for (int j = 0; j < 6; j++)
    {
        // assert(derivs[jac_offset_obj + j] && "all 7 parameters should be the same for jacobian calculation");
        for (int k = 0; k < 2; k++)
        {
            if (isnan(jout[j + k * 6]))
            {
                jout[j + k * 6] = 0;
            }
        }
        // printf("j: %d, i-1: %d \n", j, i - 1);
        // printf("j: %d, i: %d \n", j, i);
        if (false /*j == 0 || j == 3 || j == 4*/)
        {
            derivs[j][meas_idx - 1] = -jout[j];
            derivs[j][meas_idx] = -jout[j + 6];
        }
        else
        {
            derivs[j][meas_idx] = -jout[j];
            derivs[j][meas_idx - 1] = -jout[j + 6];
        }
        // printf("joff: %d, j: %d meas_idx: %d\n", jac_offset_obj, j, meas_idx);

        // printf("s: %d derivs[j=%d][meas_idx=%d] = %0.5f : pt0 %0.4f pt1 %0.4f pt2 %0.4f\n", private_data->meas[i].sensor_id, j, i - 1, jout[j], private_data->meas[i].sensor_point_3D[0], private_data->meas[i].sensor_point_3D[1], private_data->meas[i].sensor_point_3D[2]);
        // printf("s: %d derivs[j=%d][meas_idx=%d] = %0.5f\n", private_data->meas[i].sensor_id, j, i, jout[j + 6]);
        assert(isfinite(jout[j]));
        assert(isfinite(jout[j + 6]));
    }

    return 0;
}

static int mpfunc(int m, int n, FLT *p, FLT *deviates, FLT **derivs, void *p_data)
{

    mpfit_private_data *private_data = (mpfit_private_data *)p_data;
    LinmathAxisAnglePose pose = {{p[0], p[1], p[2]}, {p[3], p[4], p[5]}};

    // if (!isfinite(p[0]))
    // {
    //     return -1;
    // }
    // printf("pose in mpfit: %0.4f %0.4f %0.4f %0.4f %0.4f %0.4f \n", pose.Pos[0], pose.Pos[1], pose.Pos[2], pose.AxisAngleRot[0], pose.AxisAngleRot[1], pose.AxisAngleRot[2]);

    // LinmathAxisAnglePose *pose = new LinmathAxisAnglePose;
    // pose->Pos[0] = 0.071282573044300079;
    // pose->Pos[1] = -0.16409565508365631;
    // pose->Pos[2] = 2.7063875198364258;
    // pose->AxisAngleRot[0] = 0.6621330833988216;
    // pose->AxisAngleRot[1] = 2.4786348993027447;
    // pose->AxisAngleRot[2] = -1.3405590436508994;

    for (int i = 0; i < m; i++)
    {

        LinmathAxisAnglePose lh_P_w = {{0}, {0}}; // considering lh in origin

        LinmathPoint3d sensorPtInLH;

        ApplyAxisAnglePoseToPoint(sensorPtInLH, &pose, private_data->meas[i].sensor_point_3D);
        FLT out = SurviveReprojectAxisGen2(private_data->meas[i].lh_ch, sensorPtInLH[0], sensorPtInLH[1],
                                           sensorPtInLH[2], private_data->meas[i].axis);
        deviates[i] = (out - private_data->meas[i].pixel_pt) / sensor_variance;

        // /*  Explicit deriviative using libsurvive function

        if (derivs)
        {
            if ((i + 1) % 2 == 0)
            {
                GetExplicitJacobians(m, n, p, derivs, private_data, i, &pose, &lh_P_w);
            }
        }
        // */
    }
    /* Print explicit derivs to check if they are correct
    if (derivs)
    {
        for (int j = 0; j < 6; j++)
        {
            for (int i = 0; i < m; i++)
            {

                printf("derivs: j: %d i: %d %0.5f\n", j, i, derivs[j][i]);
            }
        }
    }
    */

    return 0;
}

PoseData CalculatePoseMPFITSingleLH(int lh, LinmathPose initial_pose)
{
    // PrintPoseLibsurviveFormatLM(lighthouses[lh].last_sync_time, "mpfit_init", &initial_pose);
    PoseData cur_pose;
    cur_pose.pose_method = MPFS;
    cur_pose.pose_method = P3P;
    const int num_pars = 6;
    mpfit_private_data mp_data;
    mp_par pars[num_pars];
    memset(&pars[0], 0, sizeof(pars));

    for (int i = 0; i < num_pars; i++)
    {
        pars[i].fixed = false;

        pars[i].limited[0] = true;
        pars[i].limited[1] = true;
        pars[i].limits[0] = (i < 3 ? -20 : -2 * PI);
        pars[i].limits[1] = -pars[i].limits[0];
        pars[i].side = 3;
        // pars[i].deriv_debug = 1;
    }
    pars[2].limits[0] = 0;

    mp_data.obj2world.Pos[0] = initial_pose.Pos[0];
    mp_data.obj2world.Pos[1] = initial_pose.Pos[1];
    mp_data.obj2world.Pos[2] = initial_pose.Pos[2];

    quattoaxisanglemag(mp_data.obj2world.AxisAngleRot, initial_pose.Rot);

    double xall[num_pars] = {initial_pose.Pos[0], initial_pose.Pos[1], initial_pose.Pos[2],
                             mp_data.obj2world.AxisAngleRot[0], mp_data.obj2world.AxisAngleRot[1],
                             mp_data.obj2world.AxisAngleRot[2]};

    // printf("xall pre: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f \n", xall[0], xall[1], xall[2], xall[3], xall[4], xall[5]);

    // cout << "active sensors: " << activations->sensor_activations[lh] << endl;

    for (int sensor = 0; sensor < NUM_TRACKER_SENSORS; sensor++)
    {
        if (activations->is_timecode_pair_valid[lh][sensor])
        {
            // printf("sensor: %d pt0: %0.4f pt1: %0.4f\n", sensor, activations->angles_raw[lh][sensor][0], activations->angles_raw[lh][sensor][1]);
            for (int ax = 0; ax < 2; ax++)
            {
                Measurement cur_measurement;
                cur_measurement.lh_ch = lh;
                cur_measurement.sensor_id = sensor;
                cur_measurement.axis = ax;
                cur_measurement.pixel_pt = activations->angles_raw[lh][sensor][ax];
                cur_measurement.sensor_point_3D[0] = tracker->model_points[sensor][0];
                cur_measurement.sensor_point_3D[1] = tracker->model_points[sensor][1];
                cur_measurement.sensor_point_3D[2] = tracker->model_points[sensor][2];
                mp_data.meas.push_back(cur_measurement);
            }
        }
    }

    if (mp_data.meas.size() < 7)
    {
        cur_pose.is_updated = false;
        cur_pose.pose = initial_pose;
        return cur_pose;
    }

    result_single_lh.resid = (double *)alloca(mp_data.meas.size() * sizeof(double));
    // result_single_lh.xerror = (double *)alloca(num_pars * sizeof(double));
    // result_single_lh.covar = (double *)alloca((num_pars * num_pars) * sizeof(double));

    int status = mpfit(mpfunc, mp_data.meas.size(), num_pars, xall, pars, &config_single_lh, (void *)&mp_data, &result_single_lh);

    // cout << "status: " << status << " iter: " << result_single_lh.niter << endl;

    double mpfit_resid = 0;
    // double mpfit_error = 0;

    // printf("Residuals:\n");
    for (int i = 0; i < mp_data.meas.size(); i++)
    {

        // mpfit_error += fabs(result_single_lh.xerror[i]);
        mpfit_resid += fabs(result_single_lh.resid[i]);
    }

    // cout << "total resid: lh: " << lh << " : " << mpfit_resid << endl;
    // cout << "total error: lh: " << lh << " : " << mpfit_error << endl;

    // if (lh_errors > 0.0001)
    // {
    //     return initial_pose;
    // }

    // printf("xall LH:%d pose: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f \n", lh, xall[0], xall[1], xall[2], xall[3], xall[4], xall[5]);

    LinmathPose est;
    est.Pos[0] = xall[0];
    est.Pos[1] = xall[1];
    est.Pos[2] = xall[2];

    LinmathAxisAngleMag mag = {xall[3], xall[4], xall[5]};

    quatfromaxisanglemag(est.Rot, mag);

    activations->last_pose_ts[lh] = lighthouses[lh].current_sync_time;

    if (signbit(est.Rot[0]) != signbit(initial_pose.Rot[0]) &&
        signbit(est.Rot[1]) != signbit(initial_pose.Rot[1]) &&
        signbit(est.Rot[2]) != signbit(initial_pose.Rot[2]) &&
        signbit(est.Rot[3]) != signbit(initial_pose.Rot[3]))

    {
        est.Rot[0] = -est.Rot[0];
        est.Rot[1] = -est.Rot[1];
        est.Rot[2] = -est.Rot[2];
        est.Rot[3] = -est.Rot[3];
    }

    lighthouses[lh].lm_error = mpfit_resid;

    cur_pose.is_updated = false;
    cur_pose.pose = est;
    return cur_pose;
}

static int mpfunc_multi(int m, int n, FLT *p, FLT *deviates, FLT **derivs, void *p_data)
{

    // if (!isfinite(p[0]))
    // {
    //     return -1;
    // }

    mpfit_private_data *private_data = (mpfit_private_data *)p_data;

    LinmathAxisAnglePose w_pose_tr = {{p[0], p[1], p[2]}, {p[3], p[4], p[5]}};

    for (int i = 0; i < m; i++)
    {
        LinmathPose lh_P_w_l = InvertPoseRtn(&lighthouses[private_data->meas[i].lh_ch].pose_in_univ);
        LinmathAxisAnglePose lh_P_w = {{lh_P_w_l.Pos[0], lh_P_w_l.Pos[1], lh_P_w_l.Pos[2]}, {0, 0, 0}};
        quattoaxisanglemag(lh_P_w.AxisAngleRot, lh_P_w_l.Rot);

        LinmathAxisAnglePose lh_P_tr;
        ApplyAxisAnglePoseToPose(&lh_P_tr, &lh_P_w, &w_pose_tr);

        LinmathPoint3d sensorPtInLH;
        ApplyAxisAnglePoseToPoint(sensorPtInLH, &lh_P_tr, private_data->meas[i].sensor_point_3D);
        FLT out = SurviveReprojectAxisGen2(private_data->meas[i].lh_ch, sensorPtInLH[0], sensorPtInLH[1],
                                           sensorPtInLH[2], private_data->meas[i].axis);
        deviates[i] = (out - private_data->meas[i].pixel_pt) / sensor_variance;

        if (derivs)
        {
            if ((i + 1) % 2 == 0)
            {
                GetExplicitJacobians(m, n, p, derivs, private_data, i, &w_pose_tr, &lh_P_w);
            }
        }
    }

    return 0;
}
PoseData CalculatePoseMPFITMultiLH(int lh, LinmathPose initial_pose)
{
    // PrintPoseLibsurviveFormatLM(lighthouses[lh].last_sync_time, "mpfit_init", &initial_pose);
    PoseData cur_pose;
    cur_pose.pose_method = MPFM;
    const int num_pars = 6;
    mpfit_private_data mp_data;
    mp_par pars[num_pars];
    memset(&pars[0], 0, sizeof(pars));

    for (int i = 0; i < num_pars; i++)
    {
        pars[i].fixed = false;
        pars[i].limited[0] = true;
        pars[i].limited[1] = true;
        pars[i].limits[0] = (i < 3 ? -20 : -2 * PI);
        pars[i].limits[1] = -pars[i].limits[0];
        pars[i].side = 3;

        // pars[i].deriv_debug = 1;
    }

    pars[2].limits[0] = 0;

    mp_data.obj2world.Pos[0] = initial_pose.Pos[0];
    mp_data.obj2world.Pos[1] = initial_pose.Pos[1];
    mp_data.obj2world.Pos[2] = initial_pose.Pos[2];

    quattoaxisanglemag(mp_data.obj2world.AxisAngleRot, initial_pose.Rot);

    double xall[num_pars] = {initial_pose.Pos[0], initial_pose.Pos[1], initial_pose.Pos[2],
                             mp_data.obj2world.AxisAngleRot[0], mp_data.obj2world.AxisAngleRot[1],
                             mp_data.obj2world.AxisAngleRot[2]};

    // printf("xall multi pre: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f \n", xall[0], xall[1], xall[2], xall[3], xall[4], xaall[5]);

    int active_sensors = 0;

    for (int l = 0; l < NUM_LH_CHANNELS; l++)
    {
        if (lighthouses[l].ootx_handler.isPoseSet &&
            lighthouses[l].current_sync_time > lighthouses[lh].last_sync_time &&
            activations->sensor_activations[l] > 2 &&
            dist3d(lighthouses[l].pose_in_univ.Pos, tracker->pose_in_universe.Pos) < 7.0)
        {

            for (int sensor = 0; sensor < NUM_TRACKER_SENSORS; sensor++)
            {
                if (activations->is_timecode_pair_valid[l][sensor] &&
                    activations->timecode[l][sensor][0] > lighthouses[lh].last_sync_time &&
                    activations->timecode[l][sensor][1] > lighthouses[lh].last_sync_time)
                {
                    for (int ax = 0; ax < 2; ax++)
                    {
                        Measurement cur_measurement;
                        cur_measurement.lh_ch = l;
                        cur_measurement.sensor_id = sensor;
                        cur_measurement.axis = ax;
                        cur_measurement.pixel_pt = activations->angles_raw[l][sensor][ax];
                        cur_measurement.sensor_point_3D[0] = tracker->model_points[sensor][0];
                        cur_measurement.sensor_point_3D[1] = tracker->model_points[sensor][1];
                        cur_measurement.sensor_point_3D[2] = tracker->model_points[sensor][2];
                        mp_data.meas.push_back(cur_measurement);
                    }
                    active_sensors++;
                }
            }
        }
    }

    // cout << "active sensors: " << active_sensors << " mpdata.meas.size: " << mp_data.meas.size() << endl;
    if (mp_data.meas.size() < 7)
    {
        cur_pose.is_updated = false;
        cur_pose.pose = initial_pose;
        return cur_pose;
    }

    result_multi_lh.resid = (double *)alloca(mp_data.meas.size() * sizeof(double));
    // result_multi_lh.xerror = (double *)alloca(num_pars * sizeof(double));
    // result_multi_lh.covar = (double *)alloca((num_pars * num_pars) * sizeof(double));

    int status = mpfit(mpfunc_multi, mp_data.meas.size(), num_pars, xall, pars, &config_multi_lh, (void *)&mp_data, &result_multi_lh);

    // cout << "status multi: " << status << " iter: " << result_multi_lh.niter << endl;

    double mpfit_resid_multi = 0;

    // printf("Residuals:\n");
    for (int i = 0; i < mp_data.meas.size(); i++)
    {

        mpfit_resid_multi += fabs(result_multi_lh.resid[i]);
    }

    tracker->lm_error = mpfit_resid_multi;
    // delete[] result_multi_lh.resid;
    // delete[] result_multi_lh.xerror;
    // delete[] result_multi_lh.covar;
    // cout << "total resid error: multi " << mpfit_resid_multi << " lh: " << lh << endl;
    // printf("xall multi pose: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f \n", xall[0], xall[1], xall[2], xall[3], xall[4], xall[5]);

    if (mpfit_resid_multi > 0.01)
    {
        cur_pose.is_updated = false;
        cur_pose.pose = initial_pose;
        return cur_pose;
    }

    LinmathPose est;
    est.Pos[0] = xall[0];
    est.Pos[1] = xall[1];
    est.Pos[2] = xall[2];

    LinmathAxisAngleMag mag = {xall[3], xall[4], xall[5]};

    quatfromaxisanglemag(est.Rot, mag);

    if (signbit(est.Rot[0]) != signbit(initial_pose.Rot[0]) &&
        signbit(est.Rot[1]) != signbit(initial_pose.Rot[1]) &&
        signbit(est.Rot[2]) != signbit(initial_pose.Rot[2]) &&
        signbit(est.Rot[3]) != signbit(initial_pose.Rot[3]))

    {
        est.Rot[0] = -est.Rot[0];
        est.Rot[1] = -est.Rot[1];
        est.Rot[2] = -est.Rot[2];
        est.Rot[3] = -est.Rot[3];
    }

    cur_pose.is_updated = false;
    cur_pose.pose = est;
    return cur_pose;
}
