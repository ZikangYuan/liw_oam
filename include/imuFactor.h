#pragma once
// c++
#include <iostream>

// eigen 
#include <Eigen/Core>

// ceres
#include <ceres/ceres.h>

// utility
#include "utility.h"

#include "imuProcessing.h"

class ImuFactor : public ceres::SizedCostFunction<18, 3, 4, 9>
{
public:
    ImuFactor(imuIntegration* pre_integration_, state* last_state_);

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    
    void check(double **parameters);

    imuIntegration* pre_integration;

    Eigen::Quaterniond rot_last;
    Eigen::Vector3d tran_last;
    Eigen::Vector3d velocity_last;
    Eigen::Vector3d ba_last;
    Eigen::Vector3d bg_last;

    static Eigen::Vector3d  t_io;
    static Eigen::Quaterniond q_io;
    static bool odom_enble;
};

class CTImuFactor : public ceres::SizedCostFunction<18, 3, 4, 9, 3, 4, 9>
{
public:
    CTImuFactor(imuIntegration* pre_integration_, int beta_);

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    
    void check(double **parameters);

    imuIntegration* pre_integration;

    static Eigen::Vector3d  t_io;
    static Eigen::Quaterniond q_io;
    static bool odom_enble;
    int beta;
};

class BeginWheelConsistencyFactor : public ceres::SizedCostFunction<3,  9>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    BeginWheelConsistencyFactor(const Eigen::Quaterniond &end_quat_ ,const Eigen::Vector3d &wheel_velocity_, double beta_);

    virtual ~BeginWheelConsistencyFactor() {}

    virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const;

    Eigen::Vector3d wheel_begin_velocity;
    Eigen::Quaterniond rot_last;

    double beta = 1.0;
};

class WheelConsistencyFactor : public ceres::SizedCostFunction<3, 4, 9>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    WheelConsistencyFactor(const Eigen::Vector3d &wheel_velocity_, double beta_);

    virtual ~WheelConsistencyFactor() {}

    virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const;

    Eigen::Vector3d wheel_velocity;
    double beta = 1.0;
};