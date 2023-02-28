#include "lioOptimization.h"

void lioOptimization::motionInitialization()
{
    bool result = false;

    if(all_cloud_frame.size() >= options.num_for_initialization)
    {
        Eigen::Vector3d g_sum = Eigen::Vector3d::Zero();

        std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> v_lidar_pose;
        v_lidar_pose.push_back(Eigen::MatrixXd::Identity(4, 4));

        Eigen::Matrix4d initial_pose = Eigen::MatrixXd::Identity(4, 4);
        initial_pose.block<3, 3>(0, 0) = all_cloud_frame[0]->p_state->rotation.toRotationMatrix();
        initial_pose.block<3, 1>(0, 3) = all_cloud_frame[0]->p_state->translation;

        for (int i = 1; i < all_cloud_frame.size(); i++)
        {
            double dt = all_cloud_frame[i]->p_state->pre_integration->sum_dt;
            Eigen::Vector3d g_temp = all_cloud_frame[i]->p_state->pre_integration->delta_v / dt;
            g_sum += g_temp;

            Eigen::Matrix4d current_pose = Eigen::MatrixXd::Identity(4, 4);
            current_pose.block<3, 3>(0, 0) = all_cloud_frame[i]->p_state->rotation.toRotationMatrix();
            current_pose.block<3, 1>(0, 3) = all_cloud_frame[i]->p_state->translation;

            Eigen::Matrix4d pose_temp = initial_pose.inverse() * current_pose;
            current_pose.block<3, 3>(0, 0) = pose_temp.block<3, 3>(0, 0) * R_imu_lidar;
            current_pose.block<3, 1>(0, 3) = R_imu_lidar.transpose() * (pose_temp.block<3, 3>(0, 0) * t_imu_lidar + pose_temp.block<3, 1>(0, 3) - t_imu_lidar);

            v_lidar_pose.push_back(current_pose);
        }

        Eigen::Vector3d g_average;
        g_average = g_sum * 1.0 / ((int)all_cloud_frame.size() - 1);

        double variance = 0;

        for (int i = 1; i < all_cloud_frame.size(); i++)
        {
            double dt = all_cloud_frame[i]->p_state->pre_integration->sum_dt;
            Eigen::Vector3d g_temp = all_cloud_frame[i]->p_state->pre_integration->delta_v / dt;
            variance += (g_temp - g_average).transpose() * (g_temp - g_average);
        }

        variance = sqrt(variance / ((int)all_cloud_frame.size() - 1));

        if(variance < 0.25)
        {
            ROS_INFO("IMU excitation not enouth!");
            return;
        }

        if(initialLidarStructure(v_lidar_pose))
            result = true;
        else
            ROS_INFO("misalign lidar structure with IMU");

        if(result == true)
            initial_flag = true;    
    }
}

void lioOptimization::staticInitialization(cloudFrame *p_frame)
{
    if(p_frame->frame_id == 1) time_init = p_frame->time_sweep_begin;

    if(p_frame->time_sweep_end - time_init > 3.0)
    {
        Eigen::Vector3d avg_velocity = p_frame->p_state->translation / (p_frame->time_sweep_end - time_init);

        if(G.norm() < 0.1)
        {
            Eigen::Vector3d ba_sum = Eigen::Vector3d::Zero();
            Eigen::Vector3d bg_sum = Eigen::Vector3d::Zero();

            assert(v_acc_static.size() == v_gyr_static.size());

            for (int i = 0; i < v_acc_static.size(); i++)
            {
                ba_sum += v_acc_static[i];
                bg_sum += v_gyr_static[i];
            }

            Eigen::Vector3d ba_avg = ba_sum / v_acc_static.size();
            Eigen::Vector3d bg_avg = bg_sum / v_gyr_static.size();

            for (int i = 0; i < all_cloud_frame.size(); i++)
            {
                all_cloud_frame[i]->p_state->velocity = Eigen::Vector3d::Zero();
                all_cloud_frame[i]->p_state->ba = ba_avg;
                all_cloud_frame[i]->p_state->bg = bg_avg;
                all_cloud_frame[i]->p_state->velocity_begin = Eigen::Vector3d::Zero();
                all_cloud_frame[i]->p_state->ba_begin = ba_avg;
                all_cloud_frame[i]->p_state->bg_begin = bg_avg;
                all_cloud_frame[i]->p_state->pre_integration->repropagate(all_cloud_frame[i]->p_state->ba, all_cloud_frame[i]->p_state->bg);
            }

            initial_flag = true;

            std::cout << "gravity vector g = " << G.transpose() << std::endl;
            std::cout << "gyr bias bg = " << all_cloud_frame.back()->p_state->bg.transpose() << std::endl;
            std::cout << "gyr bias ba = " << all_cloud_frame.back()->p_state->ba.transpose() << std::endl;

            std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>().swap(v_acc_static);
            std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>().swap(v_gyr_static);
        }
        else if(avg_velocity.norm() < 0.1)
        {
            Eigen::Vector3d g_sum = Eigen::Vector3d::Zero();
            Eigen::Vector3d bg_sum = Eigen::Vector3d::Zero();

            assert(v_acc_static.size() == v_gyr_static.size());

            for (int i = 0; i < v_acc_static.size(); i++)
            {
                g_sum += v_acc_static[i];
                bg_sum += v_gyr_static[i];
            }

            Eigen::Vector3d g_avg = g_sum / v_acc_static.size();
            Eigen::Vector3d bg_avg = bg_sum / v_gyr_static.size();

            Eigen::Vector3d z_axis = g_avg / g_avg.norm();
            Eigen::Vector3d e_1(1.0, 0.0, 0.0);
            Eigen::Vector3d x_axis = e_1 - z_axis * z_axis.transpose() * e_1;
            x_axis = x_axis / x_axis.norm();
            Eigen::Vector3d y_axis = numType::skewSymmetric(z_axis) * x_axis;
            Eigen::Matrix3d Ro;
            Ro.block<3, 1>(0, 0) = x_axis;
            Ro.block<3, 1>(0, 1) = y_axis;
            Ro.block<3, 1>(0, 2) = z_axis;

            Eigen::Vector3d ba_avg = g_avg - Ro * G;
            G = Ro * G;

            for (int i = 0; i < all_cloud_frame.size(); i++)
            {
                all_cloud_frame[i]->p_state->velocity = Eigen::Vector3d::Zero();
                all_cloud_frame[i]->p_state->ba = ba_avg;
                all_cloud_frame[i]->p_state->bg = bg_avg;
                all_cloud_frame[i]->p_state->velocity_begin = Eigen::Vector3d::Zero();
                all_cloud_frame[i]->p_state->ba_begin = ba_avg;
                all_cloud_frame[i]->p_state->bg_begin = bg_avg;
                all_cloud_frame[i]->p_state->pre_integration->repropagate(all_cloud_frame[i]->p_state->ba, all_cloud_frame[i]->p_state->bg);
            }

            initial_flag = true;

            std::cout << "gravity vector g = " << G.transpose() << std::endl;
            std::cout << "gyr bias bg = " << all_cloud_frame.back()->p_state->bg.transpose() << std::endl;
            std::cout << "gyr bias ba = " << all_cloud_frame.back()->p_state->ba.transpose() << std::endl;

            std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>().swap(v_acc_static);
            std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>().swap(v_gyr_static);
        }
        else
        {
            std::cout << "the hardware platform has moved, static init failed!" << std::endl;
        }
    }
    else
    {
        std::cout << "wait more IMU measurements ..." << std::endl;
    }
}

bool lioOptimization::initialLidarStructure(std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> &v_lidar_pose)
{
    Eigen::Vector3d g;
    Eigen::VectorXd x;
    
    bool result = lidarImuAlignment(v_lidar_pose, g, x);
    if(!result)
    {
        ROS_INFO("solve g failed!");
        return false;
    }

    for (int i = 0; i < all_cloud_frame.size(); i++)
        all_cloud_frame[i]->p_state->velocity = all_cloud_frame[i]->p_state->rotation * x.segment<3>(3 * i);

    g = all_cloud_frame[0]->p_state->rotation.toRotationMatrix() * R_imu_lidar * g;

    G = g;

    std::cout << "after transformation g = " << g.transpose() << std::endl;

    return true;
}

bool lioOptimization::lidarImuAlignment(std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> &v_lidar_pose,
     Eigen::Vector3d &g, Eigen::VectorXd &x)
{
    solveGyroscopeBias(v_lidar_pose);

    bool sucess = linearAlignment(v_lidar_pose, g, x);

    if(sucess)
        return true;
    else 
        return false;
}

void lioOptimization::solveGyroscopeBias(std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> &v_lidar_pose)
{
    Eigen::Matrix3d A;
    Eigen::Vector3d b;
    Eigen::Vector3d delta_bg;
    A.setZero();
    b.setZero();

    assert(v_lidar_pose.size() == all_cloud_frame.size());

    for (int i = 0; i < v_lidar_pose.size() - 1; i++)
    {
        Eigen::Matrix3d temp_A = Eigen::Matrix3d::Zero();
        Eigen::Vector3d temp_b = Eigen::Vector3d::Zero();

        Eigen::Matrix3d current_rot = v_lidar_pose[i].block<3, 3>(0, 0);
        Eigen::Matrix3d next_rot = v_lidar_pose[i + 1].block<3, 3>(0, 0);

        Eigen::Quaterniond quat(current_rot.transpose() * next_rot);
        temp_A = all_cloud_frame[i + 1]->p_state->pre_integration->jacobian.template block<3, 3>(O_R, O_BG);
        temp_b = 2 * (all_cloud_frame[i + 1]->p_state->pre_integration->delta_q.inverse() * quat).vec();

        A += temp_A.transpose() * temp_A;
        b += temp_A.transpose() * temp_b;
    }

    delta_bg = A.ldlt().solve(b);
    std::cout << "gyroscope bias initial calibration " << delta_bg.transpose() << std::endl;
    
    for (int i = 0; i < all_cloud_frame.size(); i++)
    {
        all_cloud_frame[i]->p_state->bg += delta_bg;
        all_cloud_frame[i]->p_state->pre_integration->repropagate(all_cloud_frame[i]->p_state->ba, all_cloud_frame[i]->p_state->bg);
    }       
}

bool lioOptimization::linearAlignment(std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> &v_lidar_pose, 
        Eigen::Vector3d &g, Eigen::VectorXd &x)
{
    assert(v_lidar_pose.size() == all_cloud_frame.size());

    int n_state = all_cloud_frame.size() * 3 + 3;

    Eigen::MatrixXd A{n_state, n_state};
    Eigen::VectorXd b{n_state};
    A.setZero();
    b.setZero();

    for (int i = 0; i < all_cloud_frame.size() - 1; i++)
    {
        Eigen::Matrix<double, 6, 9> temp_A = Eigen::MatrixXd::Zero(6, 9);
        Eigen::Matrix<double, 6, 1> temp_b = Eigen::MatrixXd::Zero(6, 1);

        double dt = all_cloud_frame[i + 1]->p_state->pre_integration->sum_dt;

        temp_A.block<3, 3>(0, 0) = - dt * Eigen::Matrix3d::Identity();
        temp_A.block<3, 3>(0, 6) = v_lidar_pose[i].block<3, 3>(0, 0).transpose() * dt * dt / 2;
        temp_b.block<3, 1>(0, 0) = all_cloud_frame[i + 1]->p_state->pre_integration->delta_p + v_lidar_pose[i].block<3, 3>(0, 0).transpose() * v_lidar_pose[i + 1].block<3, 3>(0, 0) * t_imu_lidar - t_imu_lidar
                                   - v_lidar_pose[i].block<3, 3>(0, 0).transpose() * (v_lidar_pose[i + 1].block<3, 1>(0, 3) - v_lidar_pose[i].block<3, 1>(0, 3));

        temp_A.block<3, 3>(3, 0) = - Eigen::Matrix3d::Identity();
        temp_A.block<3, 3>(3, 3) = v_lidar_pose[i].block<3, 3>(0, 0).transpose() * v_lidar_pose[i + 1].block<3, 3>(0, 0);
        temp_A.block<3, 3>(3, 6) = v_lidar_pose[i].block<3, 3>(0, 0).transpose() * dt;
        temp_b.block<3, 1>(3, 0) = all_cloud_frame[i + 1]->p_state->pre_integration->delta_v;

        Eigen::Matrix<double, 6, 6> con_inv = Eigen::MatrixXd::Identity(6, 6);

        Eigen::MatrixXd r_A = temp_A.transpose() * con_inv * temp_A;
        Eigen::VectorXd r_b = temp_A.transpose() * con_inv * temp_b;

        A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>();
        b.segment<6>(i * 3) += r_b.head<6>();

        A.bottomRightCorner<3, 3>() += r_A.bottomRightCorner<3, 3>();
        b.tail<3>() += r_b.tail<3>();

        A.block<6, 3>(i * 3, n_state - 3) += r_A.topRightCorner<6, 3>();
        A.block<3, 6>(n_state - 3, i * 3) += r_A.bottomLeftCorner<3, 6>();
    }

    A = A * 1000.0;
    b = b * 1000.0;
    x = A.ldlt().solve(b);
    g = x.segment<3>(n_state - 3);

    if(fabs(g.norm() - G.norm()) > 1.0)
    {
        return false;
    }

    refineGravity(v_lidar_pose, g, x);
    std::cout << "refine " << g.norm() << " " << g.transpose() << std::endl;

    return true;
}

Eigen::Matrix<double, 3, 2> lioOptimization::tangentBasis(Eigen::Vector3d &g0)
{
    Eigen::Vector3d b, c;
    Eigen::Vector3d a = g0.normalized();
    Eigen::Vector3d temp(0, 0, 1);
    if(a == temp)
        temp << 1, 0, 0;
    b = (temp - a * (a.transpose() * temp)).normalized();
    c = a.cross(b);
    Eigen::Matrix<double, 3, 2> bc;
    bc.block<3, 1>(0, 0) = b;
    bc.block<3, 1>(0, 1) = c;
    return bc;
}

void lioOptimization::refineGravity(std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> &v_lidar_pose, 
        Eigen::Vector3d &g, Eigen::VectorXd &x)
{
    Eigen::Vector3d g0 = g.normalized() * G.norm();
    Eigen::Vector3d lx, ly;

    int n_state = all_cloud_frame.size() * 3 + 2;

    Eigen::MatrixXd A{n_state, n_state};
    Eigen::VectorXd b{n_state};
    A.setZero();
    b.setZero();

    for (int k = 0; k < 4; k++)
    {
        Eigen::Matrix<double, 3, 2> lxly = tangentBasis(g0);

        for (int i = 0; i < all_cloud_frame.size() - 1; i++)
        {
            Eigen::Matrix<double, 6, 8> temp_A = Eigen::MatrixXd::Zero(6, 8);
            Eigen::Matrix<double, 6, 1> temp_b = Eigen::MatrixXd::Zero(6, 1);

            double dt = all_cloud_frame[i + 1]->p_state->pre_integration->sum_dt;

            temp_A.block<3, 3>(0, 0) = - dt * Eigen::Matrix3d::Identity();
            temp_A.block<3, 2>(0, 6) = v_lidar_pose[i].block<3, 3>(0, 0).transpose() * dt * dt / 2 * lxly;

            temp_b.block<3, 1>(0, 0) = all_cloud_frame[i + 1]->p_state->pre_integration->delta_p + v_lidar_pose[i].block<3, 3>(0, 0).transpose() * v_lidar_pose[i + 1].block<3, 3>(0, 0) * t_imu_lidar - t_imu_lidar 
                                     - v_lidar_pose[i].block<3, 3>(0, 0).transpose() * dt * dt / 2 * g0 - v_lidar_pose[i].block<3, 3>(0, 0).transpose() * (v_lidar_pose[i + 1].block<3, 1>(0, 3) - v_lidar_pose[i].block<3, 1>(0, 3));

            temp_A.block<3, 3>(3, 0) = - Eigen::Matrix3d::Identity();
            temp_A.block<3, 3>(3, 3) = v_lidar_pose[i].block<3, 3>(0, 0).transpose() * v_lidar_pose[i + 1].block<3, 3>(0, 0);
            temp_A.block<3, 2>(3, 6) = v_lidar_pose[i].block<3, 3>(0, 0).transpose() * dt * lxly;
            temp_b.block<3, 1>(3, 0) = all_cloud_frame[i + 1]->p_state->pre_integration->delta_v - v_lidar_pose[i].block<3, 3>(0, 0).transpose() * dt * g0;

            Eigen::Matrix<double, 6, 6> cov_inv = Eigen::Matrix<double, 6, 6>::Identity();

            Eigen::Matrix<double, 8, 8> r_A = temp_A.transpose() * cov_inv * temp_A;
            Eigen::Matrix<double, 8, 1> r_b = temp_A.transpose() * cov_inv * temp_b;

            A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>();
            b.segment<6>(i * 3) += r_b.head<6>();

            A.bottomRightCorner<2, 2>() += r_A.bottomRightCorner<2, 2>();
            b.tail<2>() += r_b.tail<2>();

            A.block<6, 2>(i * 3, n_state - 2) += r_A.topRightCorner<6, 2>();
            A.block<2, 6>(n_state - 2, i * 3) += r_A.bottomLeftCorner<2, 6>();
        }

        A = A * 1000.0;
        b = b * 1000.0;
        x = A.ldlt().solve(b);
        Eigen::VectorXd dg = x.segment<2>(n_state - 2);
        g0 = (g0 + lxly * dg).normalized() * G.norm();
    }
    g = g0;
}