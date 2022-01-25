#include "pose-manager.h"

int num_frames[NUM_LH_CHANNELS] = {0};
bool pose_initiated[NUM_LH_CHANNELS] = {false};
int num_frames_ = 0;
bool pose_initiated_ = false;
bool global_pose_init = false;

const int req_frames = 50;
double dcount = 0;
double config_lm_multi_lh_error_thresh = 0.002;

vector<LinmathPose> pose_buffer[NUM_LH_CHANNELS];

Timestamp mintsdiff = 10000;

bool unsolved_lh = false;
int num_active_lh = 0;

LinmathPose GetPoseInUniverse(int ref_lh, int cur_lh)
{

    LinmathPose tr_p_lhc = InvertPoseRtn(&activations->pose_prev[cur_lh]);
    LinmathPose lhr_p_lhc;
    ApplyPoseToPose(&lhr_p_lhc, &activations->pose_prev[ref_lh], &tr_p_lhc);

    LinmathPose w_p_lhc;
    ApplyPoseToPose(&w_p_lhc, &lighthouses[ref_lh].pose_in_univ, &lhr_p_lhc);

    // PrintPoseLibsurviveFormatLM(48000000, "lhr_p_lhc", &lhr_p_lhc);

    return w_p_lhc;
}

void AssignPoseInUniverse(int lh)
{

    if (!is_universe_known)
    {
        num_active_lh++;
        LinmathPose origin_pose = {{0}, {0}};
        origin_pose.Rot[0] = 1;
        lighthouses[lh].ootx_handler.setLHPose(origin_pose, lh);
        // writePoses(lighthouses[lh].last_sync_time, "assign", "20" + to_string(lh), &activations->pose_prev[lh]);
        lighthouses[lh].pose_in_univ = origin_pose;
        is_universe_known = true;
        unsolved_lh = false;
    }
    else
    {
        LinmathPose p_in_univ;
        for (int l = 0; l < NUM_LH_CHANNELS; l++)
        {
            if (lighthouses[l].ootx_handler.isPoseSet && l != lh && !lighthouses[lh].ootx_handler.isPoseSet)
            {
                Timestamp td = TimestampDifference(activations->last_pose_ts[lh], activations->last_pose_ts[l]);

                // cout << "td: " << td << " lhr: " << lh << " lhc:" << l << endl;

                // cout << "fabs: " << fabs(td) << " td: " << td << endl;

                if (fabs(td) < 960000 / 2)
                {
                    num_active_lh++;
                    p_in_univ = GetPoseInUniverse(l, lh);
                    lighthouses[lh].ootx_handler.setLHPose(p_in_univ, lh);
                    // writePoses(lighthouses[lh].last_sync_time, "assign", "20" + to_string(lh), &activations->pose_prev[lh]);
                    lighthouses[lh].pose_in_univ = p_in_univ;
                    unsolved_lh = false;
                    break;
                }
            }
        }
    }
}

int config_min_req_sensors_LM = 4;
double max_dist_from_lh_lm = 7.0;

PoseData CalculatePose(int lh)
{

    //  MULTI_LH_CALIB_TRACK procedure for calibration and tracking

    PoseData pose_data_single_lh;
    PoseData pose_data_multi_lh;

    if (!is_universe_known)
    {
        cur_phase = Phase_Calibration;
        if (activations->sensor_activations[lh] >= config_min_req_sensors_LM)
        {
            activations->pose_prev[lh] = CalculatePoseP3LTPReproj(lh);
            if (activations->pose_prev[lh].Pos[2] > 0.1 && !pose_initiated[lh])
            {

                num_frames[lh]++;
                if (num_frames[lh] >= req_frames)
                {
                    pose_initiated[lh] = true;
                    num_frames[lh] = 0;
                }
            }
            // sendPoseUDP("10" + to_string(lh), &activations->pose_prev[lh]);
            if (pose_initiated[lh] && !lighthouses[lh].ootx_handler.isPoseSet)
            {
                pose_data_single_lh = CalculatePoseMPFITSingleLH(lh, activations->pose_prev[lh]);
                activations->pose_prev[lh] = pose_data_single_lh.pose;
                // cout << "debug: check 1" << endl;
                if (GetDistanceFromLH(&activations->pose_prev[lh]) < max_dist_from_lh_lm && is_tracker_stationary)
                {

                    AssignPoseInUniverse(lh);
                    num_frames[lh] = 0;
                }
            }

            return pose_data_single_lh;
        }
    }

    if (is_universe_known && !lighthouses[lh].ootx_handler.isPoseSet)
    {
        cur_phase = Phase_Calibration;
        unsolved_lh = true;
        if (activations->sensor_activations[lh] >= config_min_req_sensors_LM)
        {
            activations->pose_prev[lh] = CalculatePoseP3LTPReproj(lh);
            if (activations->pose_prev[lh].Pos[2] > 0.1)
            {
                num_frames[lh]++;
                if (num_frames[lh] > req_frames)
                {
                    pose_initiated[lh] = true;
                    num_frames[lh] = 0;
                }
            }
            // sendPoseUDP("10" + to_string(lh), &activations->pose_prev[lh]);
            if (pose_initiated[lh] && !lighthouses[lh].ootx_handler.isPoseSet)
            {
                PoseData pose_data_single_lh = CalculatePoseMPFITSingleLH(lh, activations->pose_prev[lh]);
                activations->pose_prev[lh] = pose_data_single_lh.pose;
                // cout << "debug: check 2" << endl;

                pose_buffer[lh].push_back(activations->pose_prev[lh]);
                num_frames[lh]++;

                // cout << "lh: " << lh << " frame: " << num_frames[lh] << " pose: " << activations->pose_prev[lh].Pos[0] << " : " << activations->pose_prev[lh].Pos[1] << endl;

                if (GetDistanceFromLH(&activations->pose_prev[lh]) < max_dist_from_lh_lm && num_frames[lh] >= (req_frames + 1))
                {
                    LinmathPose avgpose = {{0}, {0}};

                    for (int j = 0; j < req_frames; j++)
                    {
                        // printf("idx: %d, %0.3f %0.3f %0.3f\n", j, pose_buffer[lh][j].Pos[0], pose_buffer[lh][j].Pos[1], pose_buffer[lh][j].Pos[2]);
                        avgpose.Pos[0] = avgpose.Pos[0] + pose_buffer[lh][j].Pos[0];
                        avgpose.Pos[1] = avgpose.Pos[1] + pose_buffer[lh][j].Pos[1];
                        avgpose.Pos[2] = avgpose.Pos[2] + pose_buffer[lh][j].Pos[2];

                        avgpose.Rot[0] = avgpose.Rot[0] + pose_buffer[lh][j].Rot[0];
                        avgpose.Rot[1] = avgpose.Rot[1] + pose_buffer[lh][j].Rot[1];
                        avgpose.Rot[2] = avgpose.Rot[2] + pose_buffer[lh][j].Rot[2];
                        avgpose.Rot[3] = avgpose.Rot[3] + pose_buffer[lh][j].Rot[3];
                    }

                    avgpose.Pos[0] = avgpose.Pos[0] / (double)req_frames;
                    avgpose.Pos[1] = avgpose.Pos[1] / (double)req_frames;
                    avgpose.Pos[2] = avgpose.Pos[2] / (double)req_frames;

                    avgpose.Rot[0] = avgpose.Rot[0] / (double)req_frames;
                    avgpose.Rot[1] = avgpose.Rot[1] / (double)req_frames;
                    avgpose.Rot[2] = avgpose.Rot[2] / (double)req_frames;
                    avgpose.Rot[3] = avgpose.Rot[3] / (double)req_frames;

                    activations->pose_prev[lh] = avgpose;
                }

                if (lighthouses[lh].lm_error < config_lm_multi_lh_error_thresh)
                {
                    // cout << "lh: " << lh << " lm_error: " << lighthouses[lh].lm_error << endl;

                    AssignPoseInUniverse(lh);
                    num_frames[lh] = 0;
                    unsolved_lh = false;
                }
            }

            return pose_data_single_lh;
        }
    }
    if (is_universe_known && lighthouses[lh].ootx_handler.isPoseSet && unsolved_lh)
    {
        PoseData pose_data_single_lh = CalculatePoseMPFITSingleLH(lh, activations->pose_prev[lh]);
        activations->pose_prev[lh] = pose_data_single_lh.pose;
        // cout << "debug: check 3" << endl;

        if (activations->pose_prev[lh].Pos[2] != 0 && false)
        {
            // writePosesWithError(lighthouses[lh].last_sync_time, "lm_lh", "tr_via_lh_" + to_string(lh), &activations->pose_prev[lh], lighthouses[lh].lm_error);
            if (lighthouses[lh].lm_error < config_lm_multi_lh_error_thresh)
            {
                // sendPoseUDP("10" + to_string(lh), &activations->pose_prev[lh]);
            }
            LinmathPose tr_p_lh = InvertPoseRtn(&activations->pose_prev[lh]);

            Timestamp td0 = TimestampDifference(activations->last_pose_ts[lh], activations->last_pose_ts[0]);
            // cout << "size of channel index map: " << lh_idx_ch_map.size() << endl;
            if (is_tracker_stationary)
            {
                // cout << " lh idx ch map size: " << lh_idx_ch_map.size() << endl;
                for (int i = 0; i < lh_idx_ch_map.size(); i++)
                {
                    // cout << "index: " << i << " ch: " << lh_idx_ch_map[i] << endl;
                    if (i == 0 && lh_idx_ch_map[i] == lh)
                    {
                        LinmathPose origin_pose = {{0}, {0}};
                        origin_pose.Rot[0] = 1;
                        // writePoses(lighthouses[lh].last_sync_time, "lm_lh", "ch_" + to_string(lh) + "_origin", &origin_pose);
                    }
                    else if (lh_idx_ch_map[i] != lh)
                    {
                        Timestamp td = TimestampDifference(activations->last_pose_ts[lh],
                                                           activations->last_pose_ts[lh_idx_ch_map[i]]);

                        // cout << "td: " << td << endl;
                        if (td < 960000 / 2 && lighthouses[lh_idx_ch_map[i]].ootx_handler.isPoseSet)
                        {

                            LinmathPose p_in_univ;

                            LinmathPose lhr_p_lhc;
                            ApplyPoseToPose(&lhr_p_lhc, &activations->pose_prev[lh_idx_ch_map[i]], &tr_p_lh);

                            LinmathPose w_p_lhc;
                            ApplyPoseToPose(&w_p_lhc, &lighthouses[lh_idx_ch_map[i]].pose_in_univ, &lhr_p_lhc);

                            // writePoses(lighthouses[lh].last_sync_time, "lm_lh", "ch_" + to_string(lh) + "_via_" + to_string(lh_idx_ch_map[i]), &w_p_lhc);
                        }
                        else
                        {
                            // cout << "ignoring combinaitaion ch:" << lh << " via:" << i << endl;
                        }
                    }
                }
            }
        }
    }
    if (is_universe_known && lighthouses[lh].ootx_handler.isPoseSet && tracker->pose_in_universe.Rot[0] == 0)

    {
        cur_phase = Phase_PoseEstimation;

        // std::cout << "sensor activations: " << activations->sensor_activations[lh] << endl;

        if (activations->sensor_activations[lh] >= 4)
        {
            activations->pose_prev[lh] = CalculatePoseP3LTPReproj(lh);
            // PrintPoseLibsurviveFormatLM(48000, "p3pltsol_" + to_string(lh), &tracker->pose_in_universe);
            PoseData pose_data_single_lh = CalculatePoseMPFITSingleLH(lh, activations->pose_prev[lh]);
            activations->pose_prev[lh] = pose_data_single_lh.pose;
        }

        // cout << "debug: check 4" << endl;
        LinmathPose p_in_world;
        ApplyPoseToPose(&p_in_world, &lighthouses[lh].pose_in_univ, &activations->pose_prev[lh]);
        if (GetDistanceFromLH(&p_in_world) < 7.0)
        {
            tracker->pose_in_universe = p_in_world;
            global_pose_init = true;
        }
        // PrintPoseLibsurviveFormatLM(48000, "poseinuni", &tracker->pose_in_universe);

        pose_data_single_lh.is_updated = false;
        return pose_data_single_lh;
    }

    if (is_universe_known && lighthouses[lh].ootx_handler.isPoseSet && tracker->pose_in_universe.Rot[0] != 0)
    {
        cur_phase = Phase_Tracking;
        pose_data_multi_lh = CalculatePoseMPFITMultiLH(lh, tracker->pose_in_universe);
        LinmathPose cur_est =pose_data_multi_lh.pose;
        // writePosesWithError(lighthouses[lh].last_sync_time, "lm_mutli", "tracker_pose", &tracker->pose_in_universe, tracker->lm_error);
        // dcount++;
        // tracker->pose_in_universe.Pos[0] = dcount;

        tracker->pose_in_universe.Pos[0] = cur_est.Pos[0];
        tracker->pose_in_universe.Pos[1] = cur_est.Pos[1];
        tracker->pose_in_universe.Pos[2] = cur_est.Pos[2];

        tracker->pose_in_universe.Rot[0] = cur_est.Rot[0];
        tracker->pose_in_universe.Rot[1] = cur_est.Rot[1];
        tracker->pose_in_universe.Rot[2] = cur_est.Rot[2];
        tracker->pose_in_universe.Rot[3] = cur_est.Rot[3];


        return pose_data_multi_lh;
    }
    pose_data_multi_lh.pose = tracker->pose_in_universe;
    pose_data_multi_lh.is_updated = false;
    return pose_data_multi_lh;
    //  MULTI_LH_CALIB_TRACK
}