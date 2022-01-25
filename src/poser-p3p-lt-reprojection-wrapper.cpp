#include "poser-p3p-lt-reprojection-wrapper.h"
using namespace std::chrono;

ofstream p3pltlog_rp("logs/P3PLTRANSAC_rp.txt");

LinmathPose prevEstimate_rp = {{0}, (0)};

bool valid_pose_calculated = false;

LinmathPose CalculatePoseP3LTPReproj(int lh)
{

    float dist_thresh = 0;
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> p3p_poses_list = CalculatePoseP3PLambdaTwistReproj(lh);
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> p3p_positions = p3p_poses_list.block(0, 0, p3p_poses_list.rows(), 3);
    std::vector<int> orig_indices(p3p_poses_list.rows());
    std::iota(orig_indices.begin(), orig_indices.end(), 0);

    auto start_clusteriing = high_resolution_clock::now();
    Eigen::VectorXi orig_indices_mat = Eigen::Map<Eigen::VectorXi, Eigen::Unaligned>(orig_indices.data(), orig_indices.size());
    for (int itr = 0; itr < 5; itr++)
    {

        if (p3p_positions.rows() < 2)
        {

            break;
        }

        Eigen::Matrix<float, 1, Eigen::Dynamic> p3p_center = p3p_positions.colwise().mean();

        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> p3p_position_cen = p3p_positions.rowwise() - p3p_center;

        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> p3p_position_cen_dist = p3p_position_cen.rowwise().norm();
        // cout << "position - center " << p3p_position_cen_dist << endl;
        // cout << "maxdist : " << p3p_position_cen_dist.maxCoeff() << endl;
        dist_thresh = p3p_position_cen_dist.maxCoeff() * 0.80;
        // cout << "dist_thresh " << dist_thresh << endl;

        vector<int> indices_to_keep;

        for (int row = 0; row < p3p_position_cen_dist.rows(); row++)
        {
            if (p3p_position_cen_dist(row) < dist_thresh)
            {
                indices_to_keep.push_back(row);
            }
        }

        if (indices_to_keep.size() < 1)
        {
            // cout << "indices_to_keep <1 " << endl;
            break;
        }

        // cout << "original mat: " << p3p_positions << endl;

        // https://stackoverflow.com/a/54233733 Eigen keep specific indices in rows

        p3p_positions = Eigen::MatrixXf(p3p_positions(indices_to_keep, Eigen::all));
        orig_indices_mat = Eigen::VectorXi(orig_indices_mat(indices_to_keep));
    }

    // auto stop_clustering = high_resolution_clock::now();
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> final_poses = Eigen::MatrixXf(p3p_poses_list(orig_indices_mat, Eigen::all));

    // cout << "final_poses:\n"
    //      << final_poses << endl;

    std::vector<int> selected_idx;
    int selected_idx_len = 0;
    auto start_reprojections = high_resolution_clock::now();

    for (int c1 = 0; c1 < final_poses.rows(); c1++)
    {
        bool c1_added = false;
        int cur_idx_len = 0;
        std::vector<int> cur_idx;

        float err_sum = 0;
        // cout << "c1:" << c1;
        for (int c2 = 0; c2 < final_poses.rows(); c2++)
        {
            if (c1 != c2)
            {
                Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> diffvec_o = final_poses.block(c1, 7, 1, 3) - final_poses.block(c2, 7, 1, 3);

                // cout << "absdiff: " << abs(diffvec_o.lpNorm<1>()) << endl;

                // cout << " sensors: c1:" << c1 << " c2:" << c2 << " lpnorm: " << diffvec_o.lpNorm<1>();
                Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> diffvec_p = final_poses.block(c1, 10, 1, 3) - final_poses.block(c2, 10, 1, 3);
                float error = abs(sqrt(diffvec_o.squaredNorm()) - sqrt(diffvec_p.squaredNorm()));
                // cout << " error: " << error << endl;

                if (error < 0.005)
                {
                    // cout << " c2:" << c2;
                    if (!c1_added)
                    {
                        cur_idx.push_back(c1);
                        cur_idx_len++;
                        c1_added = true;
                    }
                    cur_idx.push_back(c2);
                    cur_idx_len++;
                }
                else
                {
                    // cout << "c1: " << c1 << " c2: " << c2 << " error: " << error << " error not valid" << endl;
                }

                // cout << "error: " << error << endl;

                // cout << "fpc1: " << final_poses.block(c1, 7, 1, 3) << " fpc2: " << final_poses.block(c2, 7, 1, 3) << endl;
            }
        }
        // cout << endl;
        // cout << " err_sum: " << err_sum;

        selected_idx_len = selected_idx_len + cur_idx_len;

        selected_idx.insert(std::end(selected_idx), std::begin(cur_idx), std::end(cur_idx));
        // selected_idx = cur_idx;

        // if (err_sum < min_err_sum && err_sum > 0.00000001)
        // {
        //     // cout << "err_sum: " << err_sum << " min_err_sum:" << min_err_sum << endl;
        //     min_err_sum = err_sum;
        //     selected_idx_len = cur_idx_len;
        //     selected_idx = cur_idx;
        // }

        // if (cur_idx_len > selected_idx_len)
        // {
        //     selected_idx_len = cur_idx_len;
        //     selected_idx = cur_idx;
        // }
    }

    auto stop_reprojections = high_resolution_clock::now();

    // cout << "selected idx len: " << selected_idx_len << " final poses len: " << final_poses.rows() << endl;
    // PrintVectorInt(selected_idx);
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> selected_poses = Eigen::MatrixXf(final_poses(selected_idx, {0, 1, 2, 3, 4, 5, 6}));

    // cout << "selected_poses\n"
    //      << selected_poses << endl;

    LinmathPose estimate;
    if (selected_poses.rows() < 1)
    {
        // estimate.Pos[2] = 0;
        cout << "empty selected_poses" << endl;
        return activations->pose_prev[lh];
    }

    int num_bins = 5;

    Eigen::MatrixXf mat_sel;

    bool calc_hist = false;

    if (abs(selected_poses.col(0).minCoeff() - selected_poses.col(0).maxCoeff()) > 0.1 ||
        abs(selected_poses.col(1).minCoeff() - selected_poses.col(1).maxCoeff()) > 0.1 ||
        abs(selected_poses.col(2).minCoeff() - selected_poses.col(2).maxCoeff()) > 0.1 ||
        abs(selected_poses.col(3).minCoeff() - selected_poses.col(3).maxCoeff()) > 0.1 ||
        abs(selected_poses.col(4).minCoeff() - selected_poses.col(4).maxCoeff()) > 0.1 ||
        abs(selected_poses.col(5).minCoeff() - selected_poses.col(5).maxCoeff()) > 0.1 ||
        abs(selected_poses.col(6).minCoeff() - selected_poses.col(6).maxCoeff()) > 0.1)
    {
        calc_hist = true;
    }

    if (calc_hist)
    {

        auto h = make_histogram(axis::regular<>(num_bins, selected_poses.col(3).minCoeff(), selected_poses.col(3).maxCoeff(), "qw"),
                                axis::regular<>(num_bins, selected_poses.col(4).minCoeff(), selected_poses.col(4).maxCoeff(), "qx"),
                                axis::regular<>(num_bins, selected_poses.col(5).minCoeff(), selected_poses.col(5).maxCoeff(), "qy"),
                                axis::regular<>(num_bins, selected_poses.col(6).minCoeff(), selected_poses.col(6).maxCoeff(), "qz"));

        std::vector<float> quat4[4];

        quat4[0].resize(selected_poses.col(3).size());
        quat4[1].resize(selected_poses.col(4).size());
        quat4[2].resize(selected_poses.col(5).size());
        quat4[3].resize(selected_poses.col(6).size());
        Eigen::VectorXf::Map(&quat4[0][0], selected_poses.col(3).size()) = selected_poses.col(3);
        Eigen::VectorXf::Map(&quat4[1][0], selected_poses.col(4).size()) = selected_poses.col(4);
        Eigen::VectorXf::Map(&quat4[2][0], selected_poses.col(5).size()) = selected_poses.col(5);
        Eigen::VectorXf::Map(&quat4[3][0], selected_poses.col(6).size()) = selected_poses.col(6);

        h.fill(quat4);

        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> p3p_final_quats = selected_poses.block(0, 3, selected_poses.rows(), 4);

        std::vector<int> quat_bin_idx = {0, 0, 0, 0};
        int heighest_bin_value = 0;
        double qup[4];
        double qlo[4];

        for (auto &&x : indexed(h))
        {
            // x is a special accessor object
            const auto i = x.index(0); // current index along first axis
            const auto j = x.index(1); // current index along second axis
            const auto k = x.index(2); // current index along third axis
            const auto l = x.index(3); // current index along fourth axis
            const auto b0 = x.bin(0);  // current bin interval along first axis
            const auto b1 = x.bin(1);  // current bin interval along second axis
            const auto b2 = x.bin(2);  // current bin interval along third axis
            const auto b3 = x.bin(3);  // current bin interval along fourth axis
            const auto v = *x;         // "dereference" to get the bin value

            if (v > 0)
            {
                if (v > heighest_bin_value)
                {
                    heighest_bin_value = v;
                    quat_bin_idx = {i, j, k, l};

                    qlo[0] = b0.lower();
                    qup[0] = b0.upper();

                    qlo[1] = b1.lower();
                    qup[1] = b1.upper();

                    qlo[2] = b2.lower();
                    qup[2] = b2.upper();

                    qlo[3] = b3.lower();
                    qup[3] = b3.upper();
                }
            }
        }

        auto stop_binning = high_resolution_clock::now();

        // cout << "h: " << heighest_bin_value << " a:" << quat_bin_idx[0] << " b:" << quat_bin_idx[1] << " c:" << quat_bin_idx[2] << " d:" << quat_bin_idx[3] << endl;

        std::vector<int> keep_rows;

        // keep rows with most matching quaternionf https://stackoverflow.com/a/58353969

        for (int r = 0; r < p3p_final_quats.rows(); r++)
        {
            int matching_cols = 0;
            for (int c = 0; c < p3p_final_quats.cols(); c++)
            {

                if (p3p_final_quats(r, c) < qup[c] && p3p_final_quats(r, c) > qlo[c])
                {
                    matching_cols++;
                }
            }

            if (matching_cols == 4)
            {
                keep_rows.push_back(r);
            }
        }
        auto stop_col_select = high_resolution_clock::now();

        // cout << "function_time: " << duration_cast<milliseconds>(stop_col_select - start_P3P).count()
        //      << " binning: " << duration_cast<milliseconds>(stop_col_select - stop_reprojections).count() << endl;

        Eigen::VectorXi keep_cols = Eigen::VectorXi::LinSpaced(7, 0, 7);

        mat_sel = selected_poses(keep_rows, keep_cols);
    }
    else
    {
        mat_sel = selected_poses.block(0, 0, selected_poses.rows(), selected_poses.cols());
    }

    // cout << "mat_sel\n"
    //      << mat_sel << endl;
    if (mat_sel.rows() < 1)
    {
        // estimate.Pos[2] = 0;
        cout << "empty mat_sel" << endl;
        return activations->pose_prev[lh];
    }

    Eigen::Matrix<float, 1, Eigen::Dynamic> p3p_mean_of_selected = mat_sel.colwise().mean();

    // cout << "p3p_mean_of_selected poses:\n"
    //      << p3p_mean_of_selected << endl;
    // quatset(estimate.Rot, 1, 0, 0, 0);
    estimate.Pos[0] = p3p_mean_of_selected(0);
    estimate.Pos[1] = p3p_mean_of_selected(1);
    estimate.Pos[2] = p3p_mean_of_selected(2);
    estimate.Rot[0] = p3p_mean_of_selected(3);
    estimate.Rot[1] = p3p_mean_of_selected(4);
    estimate.Rot[2] = p3p_mean_of_selected(5);
    estimate.Rot[3] = p3p_mean_of_selected(6);

    // if (!gh_init_rp)
    // {
    //     prevEstimate_rp = estimate;
    //     gh_init_rp = true;
    // }

    // else
    // {
    //     prevEstimate_rp = estimate;
    // }

    valid_pose_calculated = true;
    return estimate;
}