#include "lidarFactor.h"

double LidarPlaneNormFactor::sqrt_info;
Eigen::Vector3d LidarPlaneNormFactor::t_il;
Eigen::Quaterniond LidarPlaneNormFactor::q_il;

double CTLidarPlaneNormFactor::sqrt_info;
Eigen::Vector3d CTLidarPlaneNormFactor::t_il;
Eigen::Quaterniond CTLidarPlaneNormFactor::q_il;

LidarPlaneNormFactor::LidarPlaneNormFactor(const Eigen::Vector3d &point_body_, const Eigen::Vector3d &norm_vector_, const double norm_offset_, double weight_)
     : point_body(point_body_), norm_vector(norm_vector_), norm_offset(norm_offset_), weight(weight_)
{

}

bool LidarPlaneNormFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{

    Eigen::Vector3d translation(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond rotation(parameters[1][3], parameters[1][0], parameters[1][1], parameters[1][2]);

    Eigen::Vector3d point_world = rotation * point_body + translation;
    double distance = norm_vector.dot(point_world) + norm_offset;

    residuals[0] = sqrt_info * weight * distance;


    if (jacobians)
    {
        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> jacobian_tran(jacobians[0]);
            jacobian_tran.setZero();

            jacobian_tran.block<1, 3>(0, 0) = sqrt_info * norm_vector.transpose() * weight;
        }
        if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor>> jacobian_rot(jacobians[1]);
            jacobian_rot.setZero();

            jacobian_rot.block<1, 3>(0, 0) = - sqrt_info * norm_vector.transpose() * rotation.toRotationMatrix() * numType::skewSymmetric(point_body) * weight;
        }
    }

    return true;
}

CTLidarPlaneNormFactor::CTLidarPlaneNormFactor(const Eigen::Vector3d &raw_keypoint_, const Eigen::Vector3d &norm_vector_, const double norm_offset_, double alpha_time_, double weight_)
     : norm_vector(norm_vector_), norm_offset(norm_offset_), alpha_time(alpha_time_), weight(weight_)
{
    raw_keypoint = q_il * raw_keypoint_ + t_il;
}

bool CTLidarPlaneNormFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{

    const Eigen::Vector3d tran_begin(parameters[0][0], parameters[0][1], parameters[0][2]);
    const Eigen::Vector3d tran_end(parameters[2][0], parameters[2][1], parameters[2][2]);
    const Eigen::Quaterniond rot_begin(parameters[1][3], parameters[1][0], parameters[1][1], parameters[1][2]);
    const Eigen::Quaterniond rot_end(parameters[3][3], parameters[3][0], parameters[3][1], parameters[3][2]);

    Eigen::Quaterniond rot_slerp = rot_begin.slerp(alpha_time, rot_end);
    rot_slerp.normalize();
    Eigen::Vector3d tran_slerp = tran_begin * (1 - alpha_time) + tran_end * alpha_time;
    Eigen::Vector3d point_world = rot_slerp * raw_keypoint + tran_slerp;

    double distance = norm_vector.dot(point_world) + norm_offset;

    residuals[0] = sqrt_info * weight * distance;


    if (jacobians)
    {
        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> jacobian_tran_begin(jacobians[0]);
            jacobian_tran_begin.setZero();

            jacobian_tran_begin.block<1, 3>(0, 0) = norm_vector.transpose() * weight * (1 - alpha_time);
            jacobian_tran_begin = sqrt_info * jacobian_tran_begin;
        }
        if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor>> jacobian_rot_begin(jacobians[1]);
            jacobian_rot_begin.setZero();

            jacobian_rot_begin.block<1, 3>(0, 0) = - norm_vector.transpose() * rot_begin.toRotationMatrix() * numType::skewSymmetric(raw_keypoint) * weight * (1 - alpha_time);
            jacobian_rot_begin = sqrt_info * jacobian_rot_begin;
        }
        if (jacobians[2])
        {
            Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> jacobian_tran_end(jacobians[2]);
            jacobian_tran_end.setZero();

            jacobian_tran_end.block<1, 3>(0, 0) = norm_vector.transpose() * weight * alpha_time;
            jacobian_tran_end = sqrt_info * jacobian_tran_end;
        }
        if (jacobians[3])
        {
            Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor>> jacobian_rot_end(jacobians[3]);
            jacobian_rot_end.setZero();

            jacobian_rot_end.block<1, 3>(0, 0) = - norm_vector.transpose() * rot_end.toRotationMatrix() * numType::skewSymmetric(raw_keypoint) * weight * alpha_time;
            jacobian_rot_end = sqrt_info * jacobian_rot_end;
        }
    }

    return true;
}

LocationConsistencyFactor::LocationConsistencyFactor(const Eigen::Vector3d &previous_location_, double beta_)
{
    previous_location = previous_location_;
    beta = beta_;
}

bool LocationConsistencyFactor::Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
{
    residuals[0] = beta * (parameters[0][0] - previous_location(0, 0));
    residuals[1] = beta * (parameters[0][1] - previous_location(1, 0));
    residuals[2] = beta * (parameters[0][2] - previous_location(2, 0));

    if (jacobians)
    {
        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacobian_tran_begin(jacobians[0]);
            jacobian_tran_begin.setZero();

            jacobian_tran_begin(0, 0) = beta;
            jacobian_tran_begin(1, 1) = beta;
            jacobian_tran_begin(2, 2) = beta;
        }
    }

    return true;
}

RotationConsistencyFactor::RotationConsistencyFactor(const Eigen::Quaterniond &previous_rotation_, double beta_)
{
    previous_rotation = previous_rotation_;
    beta = beta_;
}

bool RotationConsistencyFactor::Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
{
    Eigen::Quaterniond rot_cur(parameters[0][3], parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond q_et = previous_rotation.inverse() * rot_cur;
    Eigen::Vector3d error = 2 * q_et.vec();

    residuals[0] = error[0] * beta;
    residuals[1] = error[1] * beta;
    residuals[2] = error[2] * beta;

    if (jacobians)
    {
        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> jacobian_rot_begin(jacobians[0]);
            jacobian_rot_begin.setZero();

            jacobian_rot_begin.block<3, 3>(0, 0) = q_et.w() * Eigen::Matrix3d::Identity() + numType::skewSymmetric(q_et.vec());
            jacobian_rot_begin = jacobian_rot_begin * beta;
        }
    }

    return true;
}

SmallVelocityFactor::SmallVelocityFactor(double beta_)
{
    beta = beta_;
}

bool SmallVelocityFactor::Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
{
    residuals[0] = beta * (parameters[0][0] - parameters[1][0]);
    residuals[1] = beta * (parameters[0][1] - parameters[1][1]);
    residuals[2] = beta * (parameters[0][2] - parameters[1][2]);

    if (jacobians)
    {
        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacobian_tran_begin(jacobians[0]);
            jacobian_tran_begin.setZero();

            jacobian_tran_begin(0, 0) = beta;
            jacobian_tran_begin(1, 1) = beta;
            jacobian_tran_begin(2, 2) = beta;
        }

        if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacobian_tran_end(jacobians[1]);
            jacobian_tran_end.setZero();

            jacobian_tran_end(0, 0) = -beta;
            jacobian_tran_end(1, 1) = -beta;
            jacobian_tran_end(2, 2) = -beta;
        }
    }

    return true;
}

VelocityConsistencyFactor::VelocityConsistencyFactor(state* previous_state_, double beta_)
{
    previous_velocity = previous_state_->velocity;
    previous_ba = previous_state_->ba;
    previous_bg = previous_state_->bg;
    beta = beta_;
}

bool VelocityConsistencyFactor::Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
{
    residuals[0] = beta * (parameters[0][0] - previous_velocity(0, 0));
    residuals[1] = beta * (parameters[0][1] - previous_velocity(1, 0));
    residuals[2] = beta * (parameters[0][2] - previous_velocity(2, 0));
    residuals[3] = beta * (parameters[0][3] - previous_ba(0, 0));
    residuals[4] = beta * (parameters[0][4] - previous_ba(1, 0));
    residuals[5] = beta * (parameters[0][5] - previous_ba(2, 0));
    residuals[6] = beta * (parameters[0][6] - previous_bg(0, 0));
    residuals[7] = beta * (parameters[0][7] - previous_bg(1, 0));
    residuals[8] = beta * (parameters[0][8] - previous_bg(2, 0));

    if (jacobians)
    {
        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 9, 9, Eigen::RowMajor>> jacobian_velocity_bias_begin(jacobians[0]);
            jacobian_velocity_bias_begin.setZero();

            jacobian_velocity_bias_begin(0, 0) = beta;
            jacobian_velocity_bias_begin(1, 1) = beta;
            jacobian_velocity_bias_begin(2, 2) = beta;
            jacobian_velocity_bias_begin(3, 3) = beta;
            jacobian_velocity_bias_begin(4, 4) = beta;
            jacobian_velocity_bias_begin(5, 5) = beta;
            jacobian_velocity_bias_begin(6, 6) = beta;
            jacobian_velocity_bias_begin(7, 7) = beta;
            jacobian_velocity_bias_begin(8, 8) = beta;
        }
    }

    return true;
}

void TruncatedLoss::Evaluate(double s, double *rho) const {
    if (s < sigma2_) {
        rho[0] = s;
        rho[1] = 1.0;
        rho[2] = 0.0;
        return;
    }
    rho[0] = sigma2_;
    rho[1] = 0.0;
    rho[2] = 0.0;
}