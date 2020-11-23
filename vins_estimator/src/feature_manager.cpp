#include "feature_manager.h"

int FeaturePerId::endFrame()
{
    return start_frame + feature_per_frame.size() - 1;
}

FeatureManager::FeatureManager(Matrix3d _Rs[])
    : Rs(_Rs)
{
    for (int i = 0; i < NUM_OF_CAM; i++)
        ric[i].setIdentity();
}

void FeatureManager::setRic(Matrix3d _ric[])
{
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ric[i] = _ric[i];
    }
}

void FeatureManager::clearState()
{
    feature.clear();
}

int FeatureManager::getFeatureCount()
{
    int cnt = 0;
    for (auto &it : feature)
    {

        it.used_num = it.feature_per_frame.size();

        if (it.used_num >= 2 && it.start_frame < WINDOW_SIZE - 2)
        {
            cnt++;
        }
    }
    return cnt;
}

bool FeatureManager::addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, double td)
{
    ROS_DEBUG("input feature: %d", (int)image.size());
    ROS_DEBUG("num of feature: %d", getFeatureCount());
    double parallax_sum = 0;
    int parallax_num = 0;
    last_track_num = 0;
    for (auto &id_pts : image)
    {
        FeaturePerFrame f_per_fra(id_pts.second[0].second, td);

        int feature_id = id_pts.first;
        auto it = find_if(feature.begin(), feature.end(), [feature_id](const FeaturePerId &it) {
            return it.feature_id == feature_id;
        });

        if (it == feature.end())
        {
            feature.push_back(FeaturePerId(feature_id, frame_count));
            feature.back().feature_per_frame.push_back(f_per_fra);
        }
        else if (it->feature_id == feature_id)
        {
            it->feature_per_frame.push_back(f_per_fra);
            last_track_num++;
        }

        // depth measured points
        if (f_per_fra.measured_depth > 1.0)
        {
            it->depth_flag = 1;
        }
    }

    if (frame_count < 2 || last_track_num < 20)
        return true;

    for (auto &it_per_id : feature)
    {
        if (it_per_id.start_frame <= frame_count - 2 &&
            it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= frame_count - 1)
        {
            parallax_sum += compensatedParallax2(it_per_id, frame_count);
            parallax_num++;
        }
    }

    if (parallax_num == 0)
    {
        return true;
    }
    else
    {
        ROS_DEBUG("parallax_sum: %lf, parallax_num: %d", parallax_sum, parallax_num);
        ROS_DEBUG("current parallax: %lf", parallax_sum / parallax_num * FOCAL_LENGTH);
        return parallax_sum / parallax_num >= MIN_PARALLAX;
    }
}

void FeatureManager::debugShow()
{
    ROS_DEBUG("debug show");
    for (auto &it : feature)
    {
        ROS_ASSERT(it.feature_per_frame.size() != 0);
        ROS_ASSERT(it.start_frame >= 0);
        ROS_ASSERT(it.used_num >= 0);

        ROS_DEBUG("%d,%d,%d ", it.feature_id, it.used_num, it.start_frame);
        int sum = 0;
        for (auto &j : it.feature_per_frame)
        {
            ROS_DEBUG("%d,", int(j.is_used));
            sum += j.is_used;
            printf("(%lf,%lf) ", j.point(0), j.point(1));
        }
        ROS_ASSERT(it.used_num == sum);
    }
}

vector<pair<Vector3d, Vector3d>> FeatureManager::getCorresponding(int frame_count_l, int frame_count_r)
{
    vector<pair<Vector3d, Vector3d>> corres;
    for (auto &it : feature)
    {
        if (it.start_frame <= frame_count_l && it.endFrame() >= frame_count_r)
        {
            Vector3d a = Vector3d::Zero(), b = Vector3d::Zero();
            int idx_l = frame_count_l - it.start_frame;
            int idx_r = frame_count_r - it.start_frame;

            a = it.feature_per_frame[idx_l].point;

            b = it.feature_per_frame[idx_r].point;

            corres.push_back(make_pair(a, b));
        }
    }
    return corres;
}

void FeatureManager::setDepth(const VectorXd &x)
{
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        it_per_id.estimated_depth = 1.0 / x(++feature_index);
        //ROS_INFO("feature id %d , start_frame %d, depth %f ", it_per_id->feature_id, it_per_id-> start_frame, it_per_id->estimated_depth);
        if (it_per_id.estimated_depth < 0)
        {
            it_per_id.solve_flag = 2;
        }
        else
            it_per_id.solve_flag = 1;
    }
}

void FeatureManager::removeFailures()
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;
        if (it->solve_flag == 2)
            feature.erase(it);
    }
}

void FeatureManager::clearDepth(const VectorXd &x)
{
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        it_per_id.estimated_depth = 1.0 / x(++feature_index);
    }
}

VectorXd FeatureManager::getDepthVector()
{
    VectorXd dep_vec(getFeatureCount());
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
#if 1
        dep_vec(++feature_index) = 1. / it_per_id.estimated_depth;
#else
        dep_vec(++feature_index) = it_per_id->estimated_depth;
#endif
    }
    return dep_vec;
}

void FeatureManager::triangulateWithDepth(Vector3d Ps[], Vector3d tic[], Matrix3d ric[])
{
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        if (!it_per_id.depth_flag)
            continue;
        // if (it_per_id.estimated_depth > 0)
        // {
        //     //printf("has depth: %d\n", it_per_id.feature_id);
        //     continue;
        // }
        //printf("no  depth: %d\n", it_per_id.feature_id);

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        // ！！！vins里tic代表的是camera to imu in imu frame
        Eigen::Matrix4d P_c2i = Eigen::Matrix4d::Identity();
        P_c2i.block<3, 3>(0, 0) = ric[0];
        P_c2i.block<3, 1>(0, 3) = tic[0];

        Eigen::Matrix4d Pi_i2w = Eigen::Matrix4d::Identity();
        Pi_i2w.block<3, 3>(0, 0) = Rs[imu_i];
        Pi_i2w.block<3, 1>(0, 3) = Ps[imu_i];

        Eigen::Matrix4d Pi_c2w = Pi_i2w * P_c2i;
        double sum_depth = 0;
        int count = 0;

        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;

            // f[2]为1的是默认的点
            Eigen::Vector3d f = it_per_frame.point;
            if (f[2] <= 1.0)
                continue;

            if (imu_i == imu_j)
                it_per_id.estimated_depth = f[2];
            else
            {
                Eigen::Matrix4d Pj_i2w = Eigen::Matrix4d::Identity();
                Pj_i2w.block<3, 3>(0, 0) = Rs[imu_j];
                Pj_i2w.block<3, 1>(0, 3) = Ps[imu_j];

                Eigen::Matrix4d Pj_c2w = Pj_i2w * P_c2i;
                Eigen::Matrix4d P_j2i = Pi_c2w.inverse() * Pj_c2w; // w2i*j2w
                Eigen::Matrix4d P_i2j = P_j2i.inverse();

                Eigen::Vector4d fj(f[0] * f[2], f[1] * f[2], f[2], 1.0); // features in j
                Eigen::Vector4d fi = P_j2i * fj;                         // features in i
                Eigen::Vector2d fi_projected(fi[0] / fi[2], fi[1] / fi[2]);
                Eigen::Vector2d residual(it_per_id.feature_per_frame[imu_i].point.x() - fi_projected[0],
                                         it_per_id.feature_per_frame[imu_i].point.y() - fi_projected[1]);
                //if (residual.norm() < 10.0 / 460)
                { //this can also be adjust to improve performance
                    it_per_id.estimated_depth = sqrt(fi[0] * fi[0] + fi[1] * fi[1] + fi[2] * fi[2]);
                    sum_depth += it_per_id.estimated_depth;
                    count++;
                }
#if 0
                printf(">>>>>>>>>>>>>>>>>\n");
                printf("[%d_(%.1f %.1f)] %.3f %.3f\n", it_per_id.feature_id, it_per_frame.uv.x(), it_per_frame.uv.y(), f[2], it_per_id.measured_depth);
                //printf("i_points:\n", );
                printf("window_i[%d]:(%.1f %.1f)] %.3f %.3f %.3f\n", imu_i, it_per_id.feature_per_frame[0].uv.x(), it_per_id.feature_per_frame[0].uv.y(),
                       it_per_id.feature_per_frame[0].point[0], it_per_id.feature_per_frame[0].point[1], it_per_id.feature_per_frame[0].point[2]);
                printf("window_j[%d]:(%.1f %.1f)] %.3f %.3f %.3f\n", imu_j, it_per_frame.uv.x(), it_per_frame.uv.y(), f[0], f[1], f[2]);

                cout << "features in i:" << fi.transpose() << endl;
                cout << "features in j:" << fj.transpose() << endl;
                cout << "Pi_c2w:\n"
                     << Pi_c2w << endl;
                cout << "P_c2i:\n"
                     << P_c2i << endl;
                cout << "Pi_c2w:\n"
                     << Pi_c2w << endl;
                cout << "Pj_c2w:\n"
                     << Pj_c2w << endl;
                cout << "P_i2j:\n"
                     << P_i2j << endl;
                cout << "P_j2i:\n"
                     << P_j2i << endl;

                // cout << "R_i2w:\n"
                //      << R0 << endl;
                // cout << "T_i2w:\n"
                //      << t0.transpose() << endl;
                // cout << "R_j2w:\n"
                //      << R1 << endl;
                // cout << "T_j2w:\n"
                //      << t1.transpose() << endl;
                // cout << "R_j2i:\n"
                //      << R << endl;
                // cout << "T_j2i:\n"
                //      << t.transpose() << endl;
                // cout << "P(i2j):\n"
                //      << P << endl;

                printf("<<<<<<<<<<<<<<<\n");
#endif
            }
            //it_per_id.estimated_depth = sum_depth / count;

            //break;
        }
        if (sum_depth > 0)
            it_per_id.estimated_depth = sum_depth / count;
        // else
        //     it_per_id.estimated_depth = INIT_DEPTH;
    }
}

void FeatureManager::triangulate(Vector3d Ps[], Vector3d tic[], Matrix3d ric[])
{
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        if (it_per_id.estimated_depth > 0)
            continue;
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        ROS_ASSERT(NUM_OF_CAM == 1);
        Eigen::MatrixXd svd_A(2 * it_per_id.feature_per_frame.size(), 4);
        int svd_idx = 0;

        Eigen::Matrix<double, 3, 4> P0;
        Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
        Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
        P0.leftCols<3>() = Eigen::Matrix3d::Identity();
        P0.rightCols<1>() = Eigen::Vector3d::Zero();

        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;

            Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[0];
            Eigen::Matrix3d R1 = Rs[imu_j] * ric[0];
            Eigen::Vector3d t = R0.transpose() * (t1 - t0);
            Eigen::Matrix3d R = R0.transpose() * R1;
            Eigen::Matrix<double, 3, 4> P;
            P.leftCols<3>() = R.transpose();
            P.rightCols<1>() = -R.transpose() * t;

            // dlt求解3d点坐标
            // x*p3t-z*p1t = 0
            // y*p3t-z*p2t = 0
            Eigen::Vector3d f = it_per_frame.point.normalized();
            svd_A.row(svd_idx++) = f[0] * P.row(2) - f[2] * P.row(0);
            svd_A.row(svd_idx++) = f[1] * P.row(2) - f[2] * P.row(1);

            if (imu_i == imu_j)
                continue;
        }
        // 求得X=(x,y,z,w)
        ROS_ASSERT(svd_idx == svd_A.rows());
        Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A, Eigen::ComputeThinV).matrixV().rightCols<1>();
        double svd_method = svd_V[2] / svd_V[3];
        //it_per_id->estimated_depth = -b / A;
        //it_per_id->estimated_depth = svd_V[2] / svd_V[3];

        it_per_id.estimated_depth = svd_method;
        //it_per_id->estimated_depth = INIT_DEPTH;

        if (it_per_id.estimated_depth < 0.1)
        {
            it_per_id.estimated_depth = INIT_DEPTH;
        }
    }
}

void FeatureManager::removeOutlier()
{
    ROS_BREAK();
    int i = -1;
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;
        i += it->used_num != 0;
        if (it->used_num != 0 && it->is_outlier == true)
        {
            feature.erase(it);
        }
    }
}

void FeatureManager::removeStaticOutliers(map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> prev_points,
                                          map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> curr_points)
{
    cv::Mat image(cv::Size(640, 480), CV_8UC3, cv::Scalar(0, 0, 0));

    double td = 0;
    vector<int> outlier;
    for (auto &id_pts : curr_points)
    {
        int feature_id = id_pts.first;
        FeaturePerFrame f_per_fra(id_pts.second[0].second, td);

        auto id_pts2 = prev_points.find(feature_id);
        if (id_pts2 != prev_points.end())
        {
            vector<pair<int, Eigen::Matrix<double, 7, 1>>> pts = id_pts2->second;
            FeaturePerFrame f_per_fra2(id_pts2->second[0].second, td);
            double dx = (f_per_fra.uv.x() - f_per_fra2.uv.x());
            double dy = (f_per_fra.uv.y() - f_per_fra2.uv.y());
            //printf(" %d:%.2f,%.2f\n", feature_id, dx, dy);
            if (dx > 1 || dy > 1)
            {
                outlier.push_back(feature_id);
                cv::circle(image, cv::Point2f(f_per_fra.uv.x(), f_per_fra.uv.y()), 2, cv::Scalar(0, 0, 255), -1);
            }
            else
            {
                cv::circle(image, cv::Point2f(f_per_fra.uv.x(), f_per_fra.uv.y()), 2, cv::Scalar(0, 255, 0), -1);
            }
        }
    }

#ifdef SHOW_DEBUG
    cv::imshow("static_outlier", image);
    cv::waitKey(1);
#endif

    for (auto &feature_id : outlier)
    {
        //printf(" static outlier: %d", feature_id);
        auto it = find_if(feature.begin(), feature.end(), [feature_id](const FeaturePerId &it) {
            return it.feature_id == feature_id;
        });

        if (it != feature.end())
        {
            feature.erase(it);
            ROS_DEBUG("  remove static outlier:%d", feature_id);
        }
    }
}

void FeatureManager::removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P)
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
            it->start_frame--;
        else
        {
            Eigen::Vector3d uv_i = it->feature_per_frame[0].point;
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() < 2)
            {
                feature.erase(it);
                continue;
            }
            else
            {
                Eigen::Vector3d pts_i = uv_i * it->estimated_depth;
                Eigen::Vector3d w_pts_i = marg_R * pts_i + marg_P;
                Eigen::Vector3d pts_j = new_R.transpose() * (w_pts_i - new_P);
                double dep_j = pts_j(2);
                if (dep_j > 0)
                    it->estimated_depth = dep_j;
                else
                    it->estimated_depth = INIT_DEPTH;
            }
        }
        // remove tracking-lost feature after marginalize
        /*
        if (it->endFrame() < WINDOW_SIZE - 1)
        {
            feature.erase(it);
        }
        */
    }
}

void FeatureManager::removeBack()
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
            it->start_frame--;
        else
        {
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() == 0)
                feature.erase(it);
        }
    }
}

void FeatureManager::removeFront(int frame_count)
{
    for (auto it = feature.begin(), it_next = feature.begin(); it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame == frame_count)
        {
            it->start_frame--;
        }
        else
        {
            int j = WINDOW_SIZE - 1 - it->start_frame;
            if (it->endFrame() < frame_count - 1)
                continue;
            it->feature_per_frame.erase(it->feature_per_frame.begin() + j);
            if (it->feature_per_frame.size() == 0)
                feature.erase(it);
        }
    }
}

double FeatureManager::compensatedParallax2(const FeaturePerId &it_per_id, int frame_count)
{
    //check the second last frame is keyframe or not
    //parallax betwwen seconde last frame and third last frame
    const FeaturePerFrame &frame_i = it_per_id.feature_per_frame[frame_count - 2 - it_per_id.start_frame];
    const FeaturePerFrame &frame_j = it_per_id.feature_per_frame[frame_count - 1 - it_per_id.start_frame];

    double ans = 0;
    Vector3d p_j = frame_j.point;

    double u_j = p_j(0);
    double v_j = p_j(1);

    Vector3d p_i = frame_i.point;
    Vector3d p_i_comp;

    //int r_i = frame_count - 2;
    //int r_j = frame_count - 1;
    //p_i_comp = ric[camera_id_j].transpose() * Rs[r_j].transpose() * Rs[r_i] * ric[camera_id_i] * p_i;
    p_i_comp = p_i;
    double dep_i = p_i(2);
    double u_i = p_i(0) / dep_i;
    double v_i = p_i(1) / dep_i;
    double du = u_i - u_j, dv = v_i - v_j;

    double dep_i_comp = p_i_comp(2);
    double u_i_comp = p_i_comp(0) / dep_i_comp;
    double v_i_comp = p_i_comp(1) / dep_i_comp;
    double du_comp = u_i_comp - u_j, dv_comp = v_i_comp - v_j;

    ans = max(ans, sqrt(min(du * du + dv * dv, du_comp * du_comp + dv_comp * dv_comp)));

    return ans;
}