#include "poser-p3p-lt-reprojection.h"

std::vector<int> combination_3_sensors_rp;
std::vector<int> list_of_active_sensors_rp;
std::vector<std::array<int, 3>> combination_list_rp;
std::vector<LinmathPose> p3p_poses_list_rp;

int frame_num_rp = 0;
const int num_posemat_cols = 14;

void print_combination_rp(const vector<int> &v)
{
    static int count = 0;
    cout << "combination no " << (++count) << ": [ ";
    for (int i = 0; i < v.size(); ++i)
    {
        cout << v[i] << " ";
    }
    cout << "] " << endl;
}
void get_combination_rp(int offset, int k)
{
    if (k == 0)
    {
        // print_combination(combination_3_sensors);

        combination_list_rp.push_back({combination_3_sensors_rp[0], combination_3_sensors_rp[1], combination_3_sensors_rp[2]});
        return;
    }
    for (int i = offset; i <= list_of_active_sensors_rp.size() - k; ++i)
    {
        combination_3_sensors_rp.push_back(list_of_active_sensors_rp[i]);
        get_combination_rp(i + 1, k - 1);
        combination_3_sensors_rp.pop_back();
    }
}

Eigen::MatrixXf CalculatePoseP3PLambdaTwistReproj(int lh)
{
    frame_num_rp++;
    combination_3_sensors_rp.clear();
    list_of_active_sensors_rp.clear();
    combination_list_rp.clear();
    p3p_poses_list_rp.clear();

    Eigen::Matrix<float, Eigen::Dynamic, num_posemat_cols> p3p_poses_mat;

    int active_sensors = 0;
    for (int i = 0; i < NUM_TRACKER_SENSORS; i++)
    {
        if (activations->is_timecode_pair_valid[lh][i])
        {
            list_of_active_sensors_rp.push_back(i);
            active_sensors++;
        }
    }

    get_combination_rp(0, 3);

    int combination_num = 0;
    int combination_id = 0;

    for (std::array<int, 3> &c : combination_list_rp)
    {
        // cout << "com1 " << c[0] << " " << c[1] << " " << c[2] << endl;
        combination_num++;
        Eigen::Vector3d y1, y2, y3, X1, X2, X3;
        y1(0) = tan(activations->angles[lh][c[0]][0]);
        y1(1) = tan(activations->angles[lh][c[0]][1]);
        y1(2) = 1.0;
        y1.normalize();
        X1(0) = tracker->model_points[c[0]][0];
        X1(1) = tracker->model_points[c[0]][1];
        X1(2) = tracker->model_points[c[0]][2];

        y2(0) = tan(activations->angles[lh][c[1]][0]);
        y2(1) = tan(activations->angles[lh][c[1]][1]);
        y2(2) = 1.0;
        y2.normalize();
        X2(0) = tracker->model_points[c[1]][0];
        X2(1) = tracker->model_points[c[1]][1];
        X2(2) = tracker->model_points[c[1]][2];

        y3(0) = tan(activations->angles[lh][c[2]][0]);
        y3(1) = tan(activations->angles[lh][c[2]][1]);
        y3(2) = 1.0;
        y3.normalize();
        X3(0) = tracker->model_points[c[2]][0];
        X3(1) = tracker->model_points[c[2]][1];
        X3(2) = tracker->model_points[c[2]][2];

        std::vector<Eigen::Vector3d> x{y1, y2, y3};
        std::vector<Eigen::Vector3d> X{X1, X2, X3};
        // cout << "y1:" << y1.transpose() << endl;

        std::vector<CameraPose> poses;

        p3p(x, X, &poses);
        int solution_num = 0;
        for (CameraPose &pose : poses)
        {
            combination_id++;
            solution_num++;

            double err_R = (pose.R - Eigen::Matrix3d::Identity()).norm();
            double err_t = (pose.t - Eigen::Vector3d::Zero()).norm();

            // if (err_R < 1e-8 && err_t < 1e-8)
            //     cout << "No Idea why it is here..." << endl;

            // LinmathPose lpose;

            // cout << "poset: " << pose.t << endl;
            // Eigen::Vector3d x_flipped_t = flip_x * pose.t;

            // cout << "posetf: " << x_flipped_t << endl;
            Eigen::Quaterniond q(pose.R);

            // lpose.Rot[0] = q.w();
            // lpose.Rot[1] = q.x();
            // lpose.Rot[2] = q.y();
            // lpose.Rot[3] = q.z();
            // lpose.Pos[0] = pose.t(1);
            // lpose.Pos[1] = pose.t(0);
            // lpose.Pos[2] = pose.t(2);

            // p3p_poses_list_rp.push_back(lpose);

            // Lengths: Pose 7, orig_cen 3, reproj_cen 3, combination_num 1
            Eigen::Matrix<float, 1, num_posemat_cols> p3p_pose_vec;

            p3p_pose_vec(0) = pose.t(0);
            p3p_pose_vec(1) = pose.t(1);
            p3p_pose_vec(2) = pose.t(2);

            p3p_pose_vec(3) = q.w();
            p3p_pose_vec(4) = q.x();
            p3p_pose_vec(5) = q.y();
            p3p_pose_vec(6) = q.z();

            // Center of original triangle made by sensors
            p3p_pose_vec(7) = (X1(0) + X2(0) + X3(0)) / 3;
            p3p_pose_vec(8) = (X1(1) + X2(1) + X3(1)) / 3;
            p3p_pose_vec(9) = (X1(2) + X2(2) + X3(2)) / 3;

            // Center of triangle made by reprojected sensors

            // cout << "rotmat \n"
            //      << pose.R << endl;

            // cout << "quat \n"
            //      << q << endl;

            // cout << "t \n"
            //      << pose.t << endl;

            // cout << "X1\n"
            //      << X1 << endl;

            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> sensor_rt1 = (pose.R * X1) + pose.t;
            // cout << "sensor_rt1\n"
            //      << sensor_rt1 << endl;

            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> sensor_rt2 = (pose.R * X2) + pose.t;
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> sensor_rt3 = (pose.R * X3) + pose.t;
            p3p_pose_vec(10) = (sensor_rt1(0) + sensor_rt2(0) + sensor_rt3(0)) / 3;
            p3p_pose_vec(11) = (sensor_rt1(1) + sensor_rt2(1) + sensor_rt3(1)) / 3;
            p3p_pose_vec(12) = (sensor_rt1(2) + sensor_rt2(2) + sensor_rt3(2)) / 3;
            p3p_pose_vec(13) = combination_id;

            // cout << "sensor_rt1:\n"
            //      << p3p_pose_vec << endl;

            p3p_poses_mat.conservativeResize(p3p_poses_mat.rows() + 1, Eigen::NoChange);
            p3p_poses_mat.row(p3p_poses_mat.rows() - 1) = p3p_pose_vec;

            // cout << "p3p_poses_mat:\n"
            //      << p3p_poses_mat << endl;
        }
    }
    // cout << "p3p_poses_mat: " << p3p_poses_mat << endl;
    return p3p_poses_mat;
}