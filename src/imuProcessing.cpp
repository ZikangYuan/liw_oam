#include "imuProcessing.h"
Eigen::Vector3d imuIntegration::t_io;
Eigen::Quaterniond imuIntegration::q_io;
bool imuIntegration::odom_enble;
bool imuProcessing::odom_enble;

state::state(const Eigen::Quaterniond &rotation_, const Eigen::Vector3d &translation_,
             const Eigen::Vector3d &velocity_, const Eigen::Vector3d &ba_, const Eigen::Vector3d &bg_)
    : rotation{rotation_}, translation{translation_}, velocity{velocity_}, ba{ba_}, bg{bg_}
{
    dt_buf.push_back(0);
    rot_buf.push_back(rotation);
    trans_buf.push_back(translation);
    velo_buf.push_back(velocity);

    pre_integration = nullptr;
}

state::state(const state *state_temp, bool copy)
{
    if (copy)
    {
        rotation = state_temp->rotation;
        translation = state_temp->translation;

        rotation_begin = state_temp->rotation_begin;
        translation_begin = state_temp->translation_begin;

        velocity = state_temp->velocity;
        ba = state_temp->ba;
        bg = state_temp->bg;

        velocity_begin = state_temp->velocity_begin;
        ba_begin = state_temp->ba_begin;
        bg_begin = state_temp->bg_begin;

        pre_integration = new imuIntegration(state_temp->pre_integration,
                                             state_temp->pre_integration->linearized_acc, state_temp->pre_integration->linearized_gyr, state_temp->pre_integration->linearized_wheel);

        dt_buf.insert(dt_buf.end(), state_temp->dt_buf.begin(), state_temp->dt_buf.end());
        rot_buf.insert(rot_buf.end(), state_temp->rot_buf.begin(), state_temp->rot_buf.end());
        trans_buf.insert(trans_buf.end(), state_temp->trans_buf.begin(), state_temp->trans_buf.end());
        velo_buf.insert(velo_buf.end(), state_temp->velo_buf.begin(), state_temp->velo_buf.end());
        un_acc_buf.insert(un_acc_buf.end(), state_temp->un_acc_buf.begin(), state_temp->un_acc_buf.end());
        un_omega_buf.insert(un_omega_buf.end(), state_temp->un_omega_buf.begin(), state_temp->un_omega_buf.end());
    }
    else
    {
        rotation_begin = state_temp->rotation;
        translation_begin = state_temp->translation;
        velocity_begin = state_temp->velocity;
        ba_begin = state_temp->ba;
        bg_begin = state_temp->bg;

        rotation = state_temp->rotation;
        translation = state_temp->translation;
        velocity = state_temp->velocity;
        ba = state_temp->ba;
        bg = state_temp->bg;

        dt_buf.push_back(0);
        rot_buf.push_back(rotation);
        trans_buf.push_back(translation);
        velo_buf.push_back(velocity);
        un_acc_buf.push_back(state_temp->un_acc_buf.back());
        un_omega_buf.push_back(state_temp->un_omega_buf.back());

        pre_integration = nullptr;
    }
}

void state::release()
{
    if (pre_integration != nullptr)
        pre_integration->release();

    delete pre_integration;

    pre_integration = nullptr;

    std::vector<double>().swap(dt_buf);
    std::vector<Eigen::Quaterniond>().swap(rot_buf);
    std::vector<Eigen::Vector3d>().swap(trans_buf);
    std::vector<Eigen::Vector3d>().swap(velo_buf);
    std::vector<Eigen::Vector3d>().swap(un_acc_buf);
    std::vector<Eigen::Vector3d>().swap(un_omega_buf);
}

imuIntegration::imuIntegration(const Eigen::Vector3d &acc_0_, const Eigen::Vector3d &gyr_0_, const Eigen::Vector3d &velocity_0_,
                               const Eigen::Vector3d &linearized_ba_, const Eigen::Vector3d &linearized_bg_,
                               const double acc_cov_, const double gyr_cov_, const double b_acc_cov_, const double b_gyr_cov_, const double vel_cov_)
    : acc_0{acc_0_}, gyr_0{gyr_0_}, linearized_acc{acc_0_}, linearized_gyr{gyr_0_}, velocity_0{velocity_0_}, linearized_wheel{velocity_0},
      linearized_ba{linearized_ba_}, linearized_bg{linearized_bg_},
      acc_cov{acc_cov_}, gyr_cov{gyr_cov_}, b_acc_cov{b_acc_cov_}, b_gyr_cov{b_gyr_cov_}, vel_cov{vel_cov_},
      jacobian{Eigen::Matrix<double, 18, 18>::Identity()}, covariance{Eigen::Matrix<double, 18, 18>::Zero()},
      sum_dt{0.0}, delta_p{Eigen::Vector3d::Zero()}, delta_q{Eigen::Quaterniond::Identity()}, delta_v{Eigen::Vector3d::Zero()}, delta_g{Eigen::Vector3d::Zero()},
      delta_p_wheel{Eigen::Vector3d::Zero()}, v_wheel_last{velocity_0}
{
    noise = Eigen::Matrix<double, 24, 24>::Zero();

    noise.block<3, 3>(0, 0) = (acc_cov * acc_cov) * Eigen::Matrix3d::Identity();
    noise.block<3, 3>(3, 3) = (gyr_cov * gyr_cov) * Eigen::Matrix3d::Identity();
    noise.block<3, 3>(6, 6) = (acc_cov * acc_cov) * Eigen::Matrix3d::Identity();
    noise.block<3, 3>(9, 9) = (gyr_cov * gyr_cov) * Eigen::Matrix3d::Identity();
    noise.block<3, 3>(12, 12) = (vel_cov * vel_cov) * Eigen::Matrix3d::Identity();
    noise.block<3, 3>(15, 15) = (vel_cov * vel_cov) * Eigen::Matrix3d::Identity();
    noise.block<3, 3>(18, 18) = (b_acc_cov * b_acc_cov) * Eigen::Matrix3d::Identity();
    noise.block<3, 3>(21, 21) = (b_gyr_cov * b_gyr_cov) * Eigen::Matrix3d::Identity();
}

imuIntegration::imuIntegration(const imuIntegration *integration_temp, const Eigen::Vector3d &linearized_acc_, const Eigen::Vector3d &linearized_gyr_, const Eigen::Vector3d &linearized_wheel_)
    : linearized_acc{linearized_acc_}, linearized_gyr{linearized_gyr_}, linearized_wheel{linearized_wheel_}
{
    noise = integration_temp->noise;

    sum_dt = integration_temp->sum_dt;
    delta_p = integration_temp->delta_p;
    delta_q = integration_temp->delta_q;
    delta_v = integration_temp->delta_v;
    delta_g = integration_temp->delta_g;

    delta_p_wheel = integration_temp->delta_p_wheel;
    v_wheel_last = integration_temp->v_wheel_last;

    velocity_0 = integration_temp->velocity_0;
    velocity_1 = integration_temp->velocity_1;

    linearized_ba = integration_temp->linearized_ba;
    linearized_bg = integration_temp->linearized_bg;

    jacobian = integration_temp->jacobian;
    covariance = integration_temp->covariance;
    step_jacobian = integration_temp->step_jacobian;
    step_V = integration_temp->step_V;
    noise = integration_temp->noise;

    sum_dt = integration_temp->sum_dt;
    delta_p = integration_temp->delta_p;
    delta_q = integration_temp->delta_q;
    delta_v = integration_temp->delta_v;
    delta_g = integration_temp->delta_g;

    delta_p_wheel = integration_temp->delta_p_wheel;
    v_wheel_last = integration_temp->v_wheel_last;

    dt_buf.insert(dt_buf.end(), integration_temp->dt_buf.begin(), integration_temp->dt_buf.end());
    acc_buf.insert(acc_buf.end(), integration_temp->acc_buf.begin(), integration_temp->acc_buf.end());
    gyr_buf.insert(gyr_buf.end(), integration_temp->gyr_buf.begin(), integration_temp->gyr_buf.end());
    wheel_buf.insert(wheel_buf.end(), integration_temp->wheel_buf.begin(), integration_temp->wheel_buf.end());

    acc_cov = integration_temp->acc_cov;
    gyr_cov = integration_temp->gyr_cov;
    b_acc_cov = integration_temp->b_acc_cov;
    b_gyr_cov = integration_temp->b_gyr_cov;
    vel_cov = integration_temp->vel_cov;
}

void imuIntegration::push_back(double dt, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr, const Eigen::Vector3d &velocity)
{
    dt_buf.push_back(dt);
    acc_buf.push_back(acc);
    gyr_buf.push_back(gyr);
    wheel_buf.push_back(velocity);
    propagate(dt, acc, gyr, velocity);
}

void imuIntegration::repropagate(const Eigen::Vector3d &_linearized_ba, const Eigen::Vector3d &_linearized_bg)
{
    sum_dt = 0.0;
    acc_0 = linearized_acc;
    gyr_0 = linearized_gyr;
    delta_p.setZero();
    delta_q.setIdentity();
    delta_v.setZero();
    delta_p_wheel.setZero();

    linearized_ba = _linearized_ba;
    linearized_bg = _linearized_bg;
    jacobian.setIdentity();
    covariance.setZero();
    for (int i = 0; i < static_cast<int>(dt_buf.size()); i++)
        propagate(dt_buf[i], acc_buf[i], gyr_buf[i], wheel_buf[i]);
}

void imuIntegration::midPointIntegration(double dt_,
                                         const Eigen::Vector3d &acc_0_, const Eigen::Vector3d &gyr_0_,
                                         const Eigen::Vector3d &acc_1_, const Eigen::Vector3d &gyr_1_,
                                         const Eigen::Vector3d &_velocity_0, const Eigen::Vector3d &_velocity_1,
                                         const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q, const Eigen::Vector3d &delta_v,
                                         const Eigen::Vector3d &delta_p_wheel, const Eigen::Vector3d &v_wheel_last,
                                         const Eigen::Vector3d &linearized_ba, const Eigen::Vector3d &linearized_bg,
                                         Eigen::Vector3d &result_delta_p, Eigen::Quaterniond &result_delta_q, Eigen::Vector3d &result_delta_v,
                                         Eigen::Vector3d &result_delta_p_wheel, Eigen::Vector3d &result_v_wheel_last,
                                         Eigen::Vector3d &result_linearized_ba, Eigen::Vector3d &result_linearized_bg, bool update_jacobian)
{
    Eigen::Vector3d un_acc_0 = delta_q * (acc_0_ - linearized_ba);
    Eigen::Vector3d un_gyr = 0.5 * (gyr_0_ + gyr_1_) - linearized_bg;
    result_delta_q = delta_q * Eigen::Quaterniond(1, un_gyr(0) * dt_ / 2, un_gyr(1) * dt_ / 2, un_gyr(2) * dt_ / 2);
    Eigen::Vector3d un_acc_1 = result_delta_q * (acc_1_ - linearized_ba);
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    result_delta_p = delta_p + delta_v * dt_ + 0.5 * un_acc * dt_ * dt_;
    result_delta_v = delta_v + un_acc * dt_;
    result_linearized_ba = linearized_ba;
    result_linearized_bg = linearized_bg;

    // wheel handle
    Eigen::Vector3d un_velocity_0 = delta_q * _velocity_0;
    Eigen::Vector3d un_velocity_1 = result_delta_q * _velocity_1;
    Eigen::Vector3d un_velocity = 0.5 * (un_velocity_0 + un_velocity_1);
    result_delta_p_wheel = delta_p_wheel + un_velocity * dt_;
    result_v_wheel_last = un_velocity_1;
    // wheel handle

    if (update_jacobian)
    {
        Eigen::Vector3d omega = 0.5 * (gyr_0_ + gyr_1_) - linearized_bg;
        Eigen::Vector3d acc0_m = acc_0_ - linearized_ba;
        Eigen::Vector3d acc1_m = acc_1_ - linearized_ba;
        Eigen::Vector3d vel_0 = _velocity_0;
        Eigen::Vector3d vel_1 = _velocity_1;

        Eigen::Matrix3d R_omega_x, R_acc0_x, R_acc1_x, R_vel0_x, R_vel1_x;

        R_omega_x << 0, -omega(2), omega(1), omega(2), 0, -omega(0), -omega(1), omega(0), 0;
        R_acc0_x << 0, -acc0_m(2), acc0_m(1), acc0_m(2), 0, -acc0_m(0), -acc0_m(1), acc0_m(0), 0;
        R_acc1_x << 0, -acc1_m(2), acc1_m(1), acc1_m(2), 0, -acc1_m(0), -acc1_m(1), acc1_m(0), 0;
        R_vel0_x << 0, -vel_0(2), vel_0(1), vel_0(2), 0, -vel_0(0), -vel_0(1), vel_0(0), 0;
        R_vel1_x << 0, -vel_1(2), vel_1(1), vel_1(2), 0, -vel_1(0), -vel_1(1), vel_1(0), 0;

        Eigen::MatrixXd F_x = Eigen::MatrixXd::Zero(18, 18); // p,q,v,p_wheel,ba,bg

        F_x.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        F_x.block<3, 3>(0, 3) = -0.25 * delta_q.toRotationMatrix() * R_acc0_x * dt_ * dt_ +
                                -0.25 * result_delta_q.toRotationMatrix() * R_acc1_x * (Eigen::Matrix3d::Identity() - R_omega_x * dt_) * dt_ * dt_;
        F_x.block<3, 3>(0, 6) = Eigen::MatrixXd::Identity(3, 3) * dt_;
        F_x.block<3, 3>(0, 12) = -0.25 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * dt_ * dt_;
        F_x.block<3, 3>(0, 15) = -0.25 * result_delta_q.toRotationMatrix() * R_acc1_x * dt_ * dt_ * -dt_;

        F_x.block<3, 3>(3, 3) = Eigen::MatrixXd::Identity(3, 3) - R_omega_x * dt_;
        F_x.block<3, 3>(3, 15) = -1.0 * Eigen::MatrixXd::Identity(3, 3) * dt_;

        F_x.block<3, 3>(6, 3) = -0.5 * delta_q.toRotationMatrix() * R_acc0_x * dt_ +
                                -0.5 * result_delta_q.toRotationMatrix() * R_acc1_x * (Eigen::Matrix3d::Identity() - R_omega_x * dt_) * dt_;
        F_x.block<3, 3>(6, 6) = Eigen::MatrixXd::Identity(3, 3);
        F_x.block<3, 3>(6, 12) = -0.5 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * dt_;
        F_x.block<3, 3>(6, 15) = -0.5 * result_delta_q.toRotationMatrix() * R_acc1_x * dt_ * -dt_;

        if (odom_enble == true)
        {
            F_x.block<3, 3>(9, 3) = -0.5 * delta_q.toRotationMatrix() * R_vel0_x * dt_ +
                                    -0.5 * result_delta_q.toRotationMatrix() * R_vel1_x * (Eigen::Matrix3d::Identity() - R_omega_x * dt_) * dt_;
            F_x.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity();
            F_x.block<3, 3>(9, 12) = 0.5 * result_delta_q.toRotationMatrix() * R_vel1_x * dt_ * dt_;
        }
        // else
        // {
        //     F_x.block<3, 3>(9, 3) = Eigen::Matrix3d::Identity();
        //     F_x.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity();
        //     F_x.block<3, 3>(9, 12) = Eigen::Matrix3d::Identity();
        // }

        // F_x.block<3, 3>(9, 3) = -0.5 * delta_q.toRotationMatrix() * R_vel0_x * dt_ +
        //                         -0.5 * result_delta_q.toRotationMatrix() * R_vel1_x * (Eigen::Matrix3d::Identity() - R_omega_x * dt_) * dt_;
        // F_x.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity();
        // F_x.block<3, 3>(9, 12) = 0.5 * result_delta_q.toRotationMatrix() * R_vel1_x * dt_ * dt_;

        F_x.block<3, 3>(12, 12) = Eigen::MatrixXd::Identity(3, 3);

        F_x.block<3, 3>(15, 15) = Eigen::MatrixXd::Identity(3, 3);

        Eigen::MatrixXd F_w = Eigen::MatrixXd::Zero(18, 24);
        F_w.block<3, 3>(0, 0) = 0.25 * delta_q.toRotationMatrix() * dt_ * dt_;
        F_w.block<3, 3>(0, 3) = 0.25 * -result_delta_q.toRotationMatrix() * R_acc1_x * dt_ * dt_ * 0.5 * dt_;
        F_w.block<3, 3>(0, 6) = 0.25 * result_delta_q.toRotationMatrix() * dt_ * dt_;
        F_w.block<3, 3>(0, 9) = F_w.block<3, 3>(0, 3);

        F_w.block<3, 3>(3, 3) = 0.5 * Eigen::MatrixXd::Identity(3, 3) * dt_;
        F_w.block<3, 3>(3, 9) = 0.5 * Eigen::MatrixXd::Identity(3, 3) * dt_;

        F_w.block<3, 3>(6, 0) = 0.5 * delta_q.toRotationMatrix() * dt_;
        F_w.block<3, 3>(6, 3) = 0.5 * -result_delta_q.toRotationMatrix() * R_acc1_x * dt_ * 0.5 * dt_;
        F_w.block<3, 3>(6, 6) = 0.5 * result_delta_q.toRotationMatrix() * dt_;
        F_w.block<3, 3>(6, 9) = F_w.block<3, 3>(6, 3);

        if (odom_enble == true)
        {
            F_w.block<3, 3>(9, 3) = -0.25 * result_delta_q.toRotationMatrix() * R_vel1_x * dt_ * dt_;
            F_w.block<3, 3>(9, 9) = -0.25 * result_delta_q.toRotationMatrix() * R_vel1_x * dt_ * dt_;
            F_w.block<3, 3>(9, 12) = 0.5 * delta_q.toRotationMatrix() * dt_;
            F_w.block<3, 3>(9, 15) = 0.5 * result_delta_q.toRotationMatrix() * dt_;
        }
        // else
        // {
        //     F_w.block<3, 3>(9, 3) = Eigen::Matrix3d::Identity();
        //     F_w.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity();
        //     F_w.block<3, 3>(9, 12) = Eigen::Matrix3d::Identity();
        //     F_w.block<3, 3>(9, 15) = Eigen::Matrix3d::Identity();
        // }

        // F_w.block<3, 3>(9, 3) = -0.25 * result_delta_q.toRotationMatrix() * R_vel1_x * dt_ * dt_;
        // F_w.block<3, 3>(9, 9) = -0.25 * result_delta_q.toRotationMatrix() * R_vel1_x * dt_ * dt_;
        // F_w.block<3, 3>(9, 12) = 0.5 * delta_q.toRotationMatrix() * dt_;
        // F_w.block<3, 3>(9, 15) = 0.5 * result_delta_q.toRotationMatrix() * dt_;

        F_w.block<3, 3>(12, 18) = Eigen::MatrixXd::Identity(3, 3) * dt_;
        F_w.block<3, 3>(15, 21) = Eigen::MatrixXd::Identity(3, 3) * dt_;

        jacobian = F_x * jacobian;
        covariance = F_x * covariance * F_x.transpose() + F_w * noise * F_w.transpose();
    }
}

void imuIntegration::propagate(double dt_, const Eigen::Vector3d &acc_1_, const Eigen::Vector3d &gyr_1_, const Eigen::Vector3d &velocity_1_)
{
    dt = dt_;
    acc_1 = acc_1_;
    gyr_1 = gyr_1_;

    // wheel handle
    velocity_1 = velocity_1_;
    Eigen::Vector3d result_delta_p_wheel;
    Eigen::Vector3d result_v_wheel_last;

    Eigen::Vector3d result_delta_p;
    Eigen::Quaterniond result_delta_q;
    Eigen::Vector3d result_delta_v;
    Eigen::Vector3d result_linearized_ba;
    Eigen::Vector3d result_linearized_bg;

    midPointIntegration(dt, acc_0, gyr_0, acc_1, gyr_1, velocity_0, velocity_1, delta_p, delta_q, delta_v,
                        delta_p_wheel, v_wheel_last,
                        linearized_ba, linearized_bg,
                        result_delta_p, result_delta_q, result_delta_v,
                        result_delta_p_wheel, result_v_wheel_last,
                        result_linearized_ba, result_linearized_bg, true);

    delta_p = result_delta_p;
    delta_q = result_delta_q;
    delta_v = result_delta_v;
    linearized_ba = result_linearized_ba;
    linearized_bg = result_linearized_bg;
    delta_q.normalize();
    sum_dt += dt;
    acc_0 = acc_1;
    gyr_0 = gyr_1;

    // wheel handle
    delta_p_wheel = result_delta_p_wheel;
    v_wheel_last = result_v_wheel_last;
    velocity_0 = velocity_1;
    // wheel handle
}

Eigen::Matrix<double, 18, 1> imuIntegration::evaluate(const Eigen::Vector3d &p_last, const Eigen::Quaterniond &q_last, const Eigen::Vector3d &v_last, const Eigen::Vector3d &ba_last,
                                                      const Eigen::Vector3d &bg_last, const Eigen::Vector3d &p_cur, const Eigen::Quaterniond &q_cur, const Eigen::Vector3d &v_cur, const Eigen::Vector3d &ba_cur, const Eigen::Vector3d &bg_cur)
{
    Eigen::Matrix<double, 18, 1> residuals;

    Eigen::Matrix3d dp_dba = jacobian.block<3, 3>(O_P, O_BA);
    Eigen::Matrix3d dp_dbg = jacobian.block<3, 3>(O_P, O_BG);

    Eigen::Matrix3d dq_dbg = jacobian.block<3, 3>(O_R, O_BG);

    Eigen::Matrix3d dv_dba = jacobian.block<3, 3>(O_V, O_BA);
    Eigen::Matrix3d dv_dbg = jacobian.block<3, 3>(O_V, O_BG);

    Eigen::Matrix3d dw_dba = jacobian.block<3, 3>(O_W, O_BA);
    Eigen::Matrix3d dw_dbg = jacobian.block<3, 3>(O_W, O_BG);

    Eigen::Vector3d dba = ba_last - linearized_ba;
    Eigen::Vector3d dbg = bg_last - linearized_bg;

    Eigen::Quaterniond corrected_delta_q = delta_q * numType::deltaQ(dq_dbg * dbg);
    Eigen::Vector3d corrected_delta_v = delta_v + dv_dba * dba + dv_dbg * dbg;
    Eigen::Vector3d corrected_delta_p = delta_p + dp_dba * dba + dp_dbg * dbg;
    Eigen::Vector3d corrected_delta_p_wheel = delta_p_wheel + dw_dba * dba + dw_dbg * dbg;

    residuals.block<3, 1>(O_P, 0) = q_last.inverse() * (0.5 * G * sum_dt * sum_dt + p_cur - p_last - v_last * sum_dt) - corrected_delta_p;
    residuals.block<3, 1>(O_R, 0) = 2 * (corrected_delta_q.inverse() * (q_last.inverse() * q_cur)).vec();
    residuals.block<3, 1>(O_V, 0) = q_last.inverse() * (G * sum_dt + v_cur - v_last) - corrected_delta_v;

    if(odom_enble == true)
        residuals.block<3, 1>(O_W, 0) = q_last.inverse() * (p_cur - p_last) - t_io + q_last.inverse() * q_cur * t_io - corrected_delta_p_wheel;
    else
        residuals.block<3, 1>(O_W, 0)= Eigen::Vector3d::Zero();
    // residuals.block<3, 1>(O_W, 0) = q_last.inverse() * (p_cur - p_last) - t_io + q_last.inverse() * q_cur * t_io - corrected_delta_p_wheel;

    residuals.block<3, 1>(O_BA, 0) = ba_cur - ba_last;
    residuals.block<3, 1>(O_BG, 0) = bg_cur - bg_last;

    return residuals;
}

void imuIntegration::release()
{
    std::vector<double>().swap(dt_buf);
    std::vector<Eigen::Vector3d>().swap(acc_buf);
    std::vector<Eigen::Vector3d>().swap(gyr_buf);
    std::vector<Eigen::Vector3d>().swap(wheel_buf);
}

imuProcessing::imuProcessing()
{
    first_imu = false;

    current_state = new state(Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    last_state = nullptr;

    acc_0 = Eigen::Vector3d::Zero();
    gyr_0 = Eigen::Vector3d::Zero();
    velocity_0 = Eigen::Vector3d::Zero();
}

void imuProcessing::setAccCov(double para)
{
    acc_cov = para;
}

void imuProcessing::setGyrCov(double para)
{
    gyr_cov = para;
}

void imuProcessing::setBiasAccCov(double para)
{
    b_acc_cov = para;
}

void imuProcessing::setBiasGyrCov(double para)
{
    b_gyr_cov = para;
}

void imuProcessing::setVelCov(double para)
{
    vel_cov = para;
}

void imuProcessing::setExtrinR(Eigen::Matrix3d &R)
{
    R_imu_lidar = R;
}

void imuProcessing::setExtrinT(Eigen::Vector3d &t)
{
    t_imu_lidar = t;
}

void imuProcessing::process(double dt, const Eigen::Vector3d &linear_acceleration, const Eigen::Vector3d &angular_velocity, const Eigen::Vector3d &velocity, double timestamp)
{
    if (!first_imu)
    {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
        velocity_0 = velocity;

        Eigen::Vector3d un_acc_0_temp = current_state->rotation * (acc_0 - current_state->ba) - G;
        Eigen::Vector3d un_gyr_temp = 0.5 * (gyr_0 + angular_velocity) - current_state->bg;
        Eigen::Quaterniond rotation_temp = current_state->rotation * numType::deltaQ(un_gyr_temp * dt);
        Eigen::Vector3d un_acc_1_temp = rotation_temp * (linear_acceleration - current_state->ba) - G;
        Eigen::Vector3d un_acc_temp = 0.5 * (un_acc_0_temp + un_acc_1_temp);

        current_state->un_acc_buf.push_back(un_acc_temp);
        current_state->un_omega_buf.push_back(un_gyr_temp);

        assert(current_state->un_acc_buf.size() == current_state->trans_buf.size());
        assert(current_state->un_omega_buf.size() == current_state->rot_buf.size());
    }

    if (!current_state->pre_integration)
        current_state->pre_integration = new imuIntegration(acc_0, gyr_0, velocity, current_state->ba, current_state->bg, acc_cov, gyr_cov, b_acc_cov, b_gyr_cov, vel_cov);

    current_state->pre_integration->push_back(dt, linear_acceleration, angular_velocity, velocity);

    Eigen::Vector3d un_speed_0 = current_state->rotation * velocity_0;
    Eigen::Vector3d un_acc_0 = current_state->rotation * (acc_0 - current_state->ba) - G;

    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - current_state->bg;
    current_state->rotation *= numType::deltaQ(un_gyr * dt);
    Eigen::Vector3d un_speed_1 = current_state->rotation * velocity;
    Eigen::Vector3d un_acc_1 = current_state->rotation * (linear_acceleration - current_state->ba) - G;

    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    Eigen::Vector3d un_speed = 0.5 * (un_speed_0 + un_speed_1);

    if (odom_enble == true)
    {
        current_state->translation += dt * un_speed; // 轮速计初始状态估计
        current_state->velocity = un_speed;
    }
    else
    {
        current_state->translation += dt * current_state->velocity + 0.5 * dt * dt * un_acc; // IMU初始状态估计
        current_state->velocity += dt * un_acc;
    }

    if (optimize_finish == true)
    {
        current_state->velocity_begin_wheel = velocity;
        optimize_finish = false;
    }
    current_state->velocity_wheel = velocity;

    current_state->dt_buf.push_back(dt + current_state->dt_buf.back());
    current_state->rot_buf.push_back(current_state->rotation);
    current_state->trans_buf.push_back(current_state->translation);
    current_state->velo_buf.push_back(current_state->velocity);
    current_state->un_acc_buf.push_back(un_acc);
    current_state->un_omega_buf.push_back(un_gyr);

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
    velocity_0 = velocity;
}