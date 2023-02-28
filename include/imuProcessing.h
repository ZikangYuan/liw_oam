#pragma once
// c++
#include <iostream>

// eigen 
#include <Eigen/Core>

// ceres
#include <ceres/ceres.h>

// utility
#include "utility.h"

class imuIntegration;

class state
{
public:

	Eigen::Quaterniond rotation;
	Eigen::Vector3d translation;
	Eigen::Vector3d translation_wheel;
	Eigen::Vector3d velocity;
	Eigen::Vector3d velocity_wheel;
	Eigen::Vector3d ba;
	Eigen::Vector3d bg;
	imuIntegration *pre_integration;

	Eigen::Quaterniond rotation_begin;
	Eigen::Vector3d translation_begin;
	Eigen::Vector3d translation_begin_wheel;
	Eigen::Vector3d velocity_begin;
	Eigen::Vector3d velocity_begin_wheel;
	Eigen::Vector3d ba_begin;
	Eigen::Vector3d bg_begin;

	std::vector<double> dt_buf;
	std::vector<Eigen::Quaterniond> rot_buf;
	std::vector<Eigen::Vector3d> trans_buf;
	std::vector<Eigen::Vector3d> velo_buf;
	std::vector<Eigen::Vector3d> un_acc_buf;
	std::vector<Eigen::Vector3d> un_omega_buf;
	std::vector<Eigen::Vector3d> un_velocity_buf;

	state(const Eigen::Quaterniond &rotation_, const Eigen::Vector3d &translation_, 
		const Eigen::Vector3d &velocity_, const Eigen::Vector3d& ba_, const Eigen::Vector3d& bg_);

	state(const state* state_temp, bool copy = false);

	void release();
};

class imuIntegration
{
public:

	double dt;
    Eigen::Vector3d acc_0, gyr_0, velocity_0;
    Eigen::Vector3d acc_1, gyr_1, velocity_1;

    Eigen::Vector3d linearized_ba, linearized_bg;

    Eigen::Matrix<double, 18, 18> jacobian, covariance;
    Eigen::Matrix<double, 18, 18> step_jacobian;
    Eigen::Matrix<double, 18, 24> step_V;
    Eigen::Matrix<double, 24, 24> noise;

    double sum_dt;
    Eigen::Vector3d delta_p;
    Eigen::Quaterniond delta_q;
    Eigen::Vector3d delta_v;
    Eigen::Vector3d delta_g;

    Eigen::Vector3d delta_p_wheel;
    Eigen::Vector3d v_wheel_last;

    std::vector<double> dt_buf;
    std::vector<Eigen::Vector3d> acc_buf;
    std::vector<Eigen::Vector3d> gyr_buf;
	std::vector<Eigen::Vector3d> wheel_buf;

    double acc_cov;
	double gyr_cov;
	double b_acc_cov;
	double b_gyr_cov;
	double vel_cov;

	static Eigen::Vector3d t_io;
	static Eigen::Quaterniond q_io;
	static bool odom_enble;

	const Eigen::Vector3d linearized_acc, linearized_gyr, linearized_wheel;

	imuIntegration() = delete;
	
    imuIntegration(const Eigen::Vector3d &acc_0_, const Eigen::Vector3d &gyr_0_,
                   const Eigen::Vector3d &linearized_ba_, const Eigen::Vector3d &linearized_bg_, const Eigen::Vector3d &velocity_0_,
                   const double acc_cov_, const double gyr_cov_, const double b_acc_cov_, const double b_gyr_cov_, const double vel_cov_);

    imuIntegration(const imuIntegration* integration_temp, const Eigen::Vector3d &linearized_acc_, const Eigen::Vector3d &linearized_gyr_, const Eigen::Vector3d &linearized_wheel_);

	void midPointIntegration(double dt_, 
                            const Eigen::Vector3d &acc_0_, const Eigen::Vector3d &gyr_0_,
                            const Eigen::Vector3d &acc_1_, const Eigen::Vector3d &gyr_1_,
							const Eigen::Vector3d &_velocity_0, const Eigen::Vector3d &_velocity_1,
                            const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q, const Eigen::Vector3d &delta_v,
							const Eigen::Vector3d &delta_p_wheel, const Eigen::Vector3d &v_wheel_last,
                            const Eigen::Vector3d &linearized_ba, const Eigen::Vector3d &linearized_bg,
                            Eigen::Vector3d &result_delta_p, Eigen::Quaterniond &result_delta_q, Eigen::Vector3d &result_delta_v,
							Eigen::Vector3d &result_delta_p_wheel, Eigen::Vector3d &result_v_wheel_last,
                            Eigen::Vector3d &result_linearized_ba, Eigen::Vector3d &result_linearized_bg, bool update_jacobian);

    void propagate(double dt_, const Eigen::Vector3d &acc_1_, const Eigen::Vector3d &gyr_1_, const Eigen::Vector3d &velocity_1_);

    void push_back(double dt, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr, const Eigen::Vector3d &velocity);

    void repropagate(const Eigen::Vector3d &linearized_ba_, const Eigen::Vector3d &linearized_bg_);

    Eigen::Matrix<double, 18, 1> evaluate(const Eigen::Vector3d &p_last, const Eigen::Quaterniond &q_last, const Eigen::Vector3d &v_last, const Eigen::Vector3d &ba_last, 
	const Eigen::Vector3d &bg_last, const Eigen::Vector3d &p_cur, const Eigen::Quaterniond &q_cur, const Eigen::Vector3d &v_cur, const Eigen::Vector3d &ba_cur, const Eigen::Vector3d &bg_cur);

	void release();
};

class imuProcessing
{
private:

	double acc_cov;
	double gyr_cov;
	double b_acc_cov;
	double b_gyr_cov;
	double vel_cov;

	Eigen::Matrix3d R_imu_lidar;
	Eigen::Vector3d t_imu_lidar;
	Eigen::Matrix3d R_imu_odometer;
	Eigen::Vector3d t_imu_odometer;

	bool first_imu;

	Eigen::Vector3d acc_0, gyr_0,velocity_0;

public:

	static bool odom_enble;
	state *current_state;
	state *last_state;
	bool optimize_finish = true;
	imuProcessing();


	void setAccCov(double para);
	void setGyrCov(double para);
	void setBiasAccCov(double para);
	void setBiasGyrCov(double para);
	void setVelCov(double para);

	void setExtrinR(Eigen::Matrix3d &R);
	void setExtrinT(Eigen::Vector3d &t);
	void setExtrinR_odometer(Eigen::Matrix3d &R);
	void setExtrinT_odometer(Eigen::Vector3d &t);

	void process(double dt, const Eigen::Vector3d &linear_acceleration, const Eigen::Vector3d &angular_velocity, const Eigen::Vector3d &velocity, double timestamp);
};