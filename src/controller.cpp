#include "controller.h"
using namespace std;

LinmathPose ref_coordinate = {{0}, {0}};
bool ref_set = false;

double td_last_pose = 0;
bool ts_init = false;
Timestamp ts_last_pose = 0;
double min_pose_tdiff = 0.018; // in seconds
bool pose_every_sync = false;

void ControllerSetPoseEverySync(bool pose_every_sync)
{
	pose_every_sync = pose_every_sync;
}

LinmathPose CalibrateUniverse(LinmathPose *estdpose)
{

	LinmathPose initROt = {{0, 0, 0}, {1, 0, 0, 0}};

	LinmathQuat initQuat = {0, 0, 0, 1};

	quatcopy(initROt.Rot, initQuat);

	LinmathPose upw = {{8.72303863, -0.17235806, -8.16103763}, {0.86040305, 0.26883925, -0.40729955, 0.14676215}};

	ApplyPoseToPose(&upw, &upw, estdpose);

	upw.Pos[1] = upw.Pos[1] + 2.7;
	double tempaxis = upw.Pos[1];
	upw.Pos[1] = -upw.Pos[2];
	upw.Pos[2] = tempaxis;

	LinmathPose initROt2 = {{0, 0, 0}, {0.707, 0.707, 0, 0}};

	ApplyPoseToPose(&initROt, &initROt2, &initROt);

	quatnormalize(initROt.Rot, initROt.Rot);

	ApplyPoseToPose(&upw, &upw, &initROt);

	// x, y, z
	// w, x, y, z

	LinmathPose corrected;

	corrected.Pos[0] = upw.Pos[0];
	corrected.Pos[1] = upw.Pos[1];
	corrected.Pos[2] = upw.Pos[2];

	/** Working but 180 degree flipped on green axis
	corrected.Rot[0] = upw.Rot[0];
	corrected.Rot[1] = upw.Rot[2];
	corrected.Rot[2] = upw.Rot[3];
	corrected.Rot[3] = -upw.Rot[1];

	*/

	LinmathPose flipaxis = {{0, 0, 0}, {0, 1, 0, 0}};

	corrected.Rot[0] = upw.Rot[0];
	corrected.Rot[1] = upw.Rot[2];
	corrected.Rot[2] = -upw.Rot[3];
	corrected.Rot[3] = upw.Rot[1];

	ApplyPoseToPose(&corrected, &corrected, &flipaxis);

	quatnormalize(corrected.Rot, corrected.Rot);

	return corrected;
}

void ControllerSweep(SWEEP sweep)
{
	if (lighthouses[sweep.channel].last_sync_time > 0 && sweep.time > lighthouses[sweep.channel].last_sync_time)
	{
		lighthouses[sweep.channel].light_frame.timecode[sweep.sensor][lighthouses[sweep.channel].light_frame.sweeps_in_cycle[sweep.sensor]] = sweep.time;
		lighthouses[sweep.channel].light_frame.sweeps_in_cycle[sweep.sensor]++;
	}
}

SpreePose calculated_pose = {{0}, {0}};

LinmathPose estimate;

SpreePoseData ControllerSync(SYNC sync)
{

	SpreePose calculated_pose = {{0}, {0}};
	SpreePoseData spree_pose_data;
	// if (sync.channel != 4) // working with single channel only
	// {
	// 	return;
	// }

	// cout << "sync" << endl;

	double time_since_last_pose = 0;

	if (!ts_init)
	{
		ts_last_pose = sync.time;
		ts_init = true;
	}
	Timestamp tdc = TimestampDifference(sync.time, ts_last_pose);
	time_since_last_pose = (double)tdc / TRACKER_CLK_FREQ;

	if (!lighthouses[sync.channel].ootx_handler.isOOTXInfoAvailable(sync.channel))
	{
		lighthouses[sync.channel].ootx_handler.addBit(sync.ootx);
		calculated_pose.Pos[2] = -2; // TODO better way to control not ready poses, e.g. through return value and tryGet  pattern
		spree_pose_data.pose = calculated_pose;
		spree_pose_data.pose_method = P3P;
		spree_pose_data.is_updated = false;
		return spree_pose_data;
	}

	lighthouses[sync.channel].current_sync_time = sync.time;

	ProcessFrame(sync.channel);

	// ProcessFrame(sync.channel);

	// cout << "dis diff: " << dist3d(lighthouses[sync.channel].pose_in_univ.Pos, tracker->pose_in_universe.Pos) << endl;

	if (cur_phase != Phase_Tracking)
	{
		PoseData pose_data_est = CalculatePose(sync.channel);
		LinmathPose estimate = pose_data_est.pose;

		if (estimate.Pos[2] != 0)
		{
			// setImagePoseString(sync.channel, &estimate);
			// setImageDistanceString(sync.channel, &estimate);
			if (true)
			{
				if (cur_phase != Phase_Tracking)
				{
					// sendPoseUDP("20" + to_string(sync.channel), &estimate);
				}
				else
				{
					// sendPoseUDP("31", &estimate);
				}
			}
		}
	}

	else if (time_since_last_pose > min_pose_tdiff || pose_every_sync)
	{
		// cout << "distance of lh: " << (unsigned)sync.channel << " univ pose from tracker : " << dist3d(lighthouses[sync.channel].pose_in_univ.Pos, tracker->pose_in_universe.Pos) << endl;
		// if (getBoolFromConfig(config_show_lh_pose) && lighthouses[sync.channel].ootx_handler.isPoseSet)
		// {
		// 	sendPoseUDP("4" + to_string(sync.channel), &lighthouses[sync.channel].pose_in_univ);
		// }
		cout << "lh: " << (unsigned)sync.channel << " time since last pose: " << time_since_last_pose << " tdc: " << tdc << endl;
		PoseData pose_data_est = CalculatePose(sync.channel);
		LinmathPose estimate = pose_data_est.pose;

		if (estimate.Pos[2] != 0)
		{
			// setImagePoseString(sync.channel, &estimate);
			// setImageDistanceString(sync.channel, &estimate);
			if (true)
			{
				if (cur_phase != Phase_Tracking)
				{
					// sendPoseUDP("20" + to_string(sync.channel), &estimate);
				}
				else
				{
					lighthouses[sync.channel].last_sync_time = lighthouses[sync.channel].current_sync_time;

					ts_last_pose = lighthouses[sync.channel].last_sync_time;

					LinmathPose poseInU = CalibrateUniverse(&tracker->pose_in_universe);

					calculated_pose.Pos[0] = poseInU.Pos[0];
					calculated_pose.Pos[1] = poseInU.Pos[1];
					calculated_pose.Pos[2] = poseInU.Pos[2];

					calculated_pose.Rot[0] = poseInU.Rot[0];
					calculated_pose.Rot[1] = poseInU.Rot[1];
					calculated_pose.Rot[2] = poseInU.Rot[2];
					calculated_pose.Rot[3] = poseInU.Rot[3];

					// calculated_pose.Pos[0] = tracker->lm_error;

					// PrintPoseLibsurviveFormatLM(480000, "tracking", &tracker->pose_in_universe);

					spree_pose_data.pose = calculated_pose;
					spree_pose_data.is_updated = pose_data_est.is_updated;
					spree_pose_data.pose_method = pose_data_est.pose_method;
					return spree_pose_data;
				}
			}
			// if (getBoolFromConfig(config_print_poses))
			// {
			// 	PrintPoseLibsurviveFormatLM(sync.time, "20" + to_string(sync.channel), &estimate);
			// }
		}
	}

	lighthouses[sync.channel].last_sync_time = lighthouses[sync.channel].current_sync_time;

	LinmathPose poseInU = CalibrateUniverse(&tracker->pose_in_universe);

	calculated_pose.Pos[0] = poseInU.Pos[0];
	calculated_pose.Pos[1] = poseInU.Pos[1];
	calculated_pose.Pos[2] = poseInU.Pos[2];

	calculated_pose.Rot[0] = poseInU.Rot[0];
	calculated_pose.Rot[1] = poseInU.Rot[1];
	calculated_pose.Rot[2] = poseInU.Rot[2];
	calculated_pose.Rot[3] = poseInU.Rot[3];

	spree_pose_data.pose = calculated_pose;
	spree_pose_data.is_updated = false;
	spree_pose_data.pose_method = P3P;
	return spree_pose_data;
}

void ControllerIMU(IMU imu)
{

	activations->last_imu = imu.time;

	activations->accel[0] = imu.acc.x;
	activations->accel[1] = imu.acc.y;
	activations->accel[2] = imu.acc.z;

	activations->gyro[0] = imu.gyro.x;
	activations->gyro[1] = imu.gyro.y;
	activations->gyro[2] = imu.gyro.z;
}
