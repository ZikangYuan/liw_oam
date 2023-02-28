#pragma once
// c++
#include <iostream>
#include <string>
#include <tr1/unordered_map>

// ros
#include <sensor_msgs/PointCloud2.h>

// eigen 
#include <Eigen/Core>

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

// cloud processing
#include "cloudProcessing.h"

#include "cloudMap.h"


extern bool debug_output;
extern std::string output_path;

extern bool time_diff_enable;
extern double time_diff;

extern float mov_threshold;

extern bool initial_flag;

extern Eigen::Vector3d G;

bool time_list(point3D &point_1, point3D &point_2);

bool time_list_velodyne(velodyne_ros::Point &point_1, velodyne_ros::Point &point_2);

void point3DtoPCL(std::vector<point3D> &v_point_temp, pcl::PointCloud<pcl::PointXYZINormal>::Ptr &p_cloud_temp);

Eigen::Matrix3d mat33FromArray(std::vector<double> &array);

Eigen::Vector3d vec3FromArray(std::vector<double> &array);

void pointBodyToWorld(pcl::PointXYZINormal const * const pi, pcl::PointXYZINormal * const po, Eigen::Matrix3d &R_world_cur, 
    Eigen::Vector3d &t_world_cur, Eigen::Matrix3d &R_imu_lidar, Eigen::Vector3d &t_imu_lidar);

void RGBpointLidarToIMU(pcl::PointXYZINormal const * const pi, pcl::PointXYZINormal * const po, Eigen::Matrix3d &R_imu_lidar, Eigen::Vector3d &t_imu_lidar);

bool planeFitting(Eigen::Matrix<double, 4, 1> &plane_parameter, const std::vector<pcl::PointXYZINormal, Eigen::aligned_allocator<pcl::PointXYZINormal>> &point, const double &threshold);

double AngularDistance(const Eigen::Matrix3d &rota, const Eigen::Matrix3d &rotb);

double AngularDistance(const Eigen::Quaterniond &q_a, const Eigen::Quaterniond &q_b);

float calculateDist2(pcl::PointXYZINormal point1, pcl::PointXYZINormal point2);

void saveCutCloud(std::string &str, pcl::PointCloud<pcl::PointXYZINormal>::Ptr &p_cloud_temp);

enum SizeParameterization
{
    SIZE_POSE = 7,
    SIZE_SPEEDBIAS = 9, 
    NUM_MATCH_POINTS = 20
};

enum StateOrder
{
    O_P = 0,
    O_R = 3,
    O_V = 6,
    O_W = 9,
    O_BA = 12,
    O_BG = 15
};

enum NoiseOrder
{
    O_AN = 0,
    O_GN = 3,
    O_AW = 6,
    O_GW = 9
};

enum MotionCompensation
{
    NONE = 0,
    CONSTANT_VELOCITY = 1,
    ITERATIVE = 2,
    CONTINUOUS = 3
};

enum StateInitialization
{
    INIT_NONE = 0,
    INIT_CONSTANT_VELOCITY = 1, 
    INIT_IMU = 2
};

enum LioInitialization
{
    MOTION_INIT = 0,
    STATIC_INIT = 1
};

enum IcpDistance
{
    POINT_TO_PLANE = 0,
    CT_POINT_TO_PLANE = 1
};

enum SolveMethod
{
    LIO = 0,
    LIDAR = 1
};

enum LeastSquares
{
    STANDARD = 0,
    CAUCHY = 1,
    HUBER = 2,
    TOLERANT = 3,
    TRUNCATED = 4
};

enum VizMode
{
    TIMESTAMP = 0,
    WEIGHT = 1,
    NORMAL = 2
};

enum WeightingScheme
{
    PLANARITY = 0,      // Weighs residuals by their planarity coefficient
    NEIGHBORHOOD = 1,   // Weighs residuals by the distance to their neares neighbors
    ALL = 2             // Combines all weighting schemes with different coefficients
};

class numType
{
  public:
    template <typename Derived>
    static Eigen::Quaternion<typename Derived::Scalar> deltaQ(const Eigen::MatrixBase<Derived> &theta)
    {
        typedef typename Derived::Scalar Scalar_t;

        Eigen::Quaternion<Scalar_t> dq;
        Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
        half_theta /= static_cast<Scalar_t>(2.0);
        dq.w() = static_cast<Scalar_t>(1.0);
        dq.x() = half_theta.x();
        dq.y() = half_theta.y();
        dq.z() = half_theta.z();
        dq.normalize();
        return dq;
    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetric(const Eigen::MatrixBase<Derived> &mat)
    {
        Eigen::Matrix<typename Derived::Scalar, 3, 3> mat_skew;
        mat_skew << typename Derived::Scalar(0), -mat(2), mat(1),
            mat(2), typename Derived::Scalar(0), -mat(0),
            -mat(1), mat(0), typename Derived::Scalar(0);
        return mat_skew;
    }

    template <typename Derived>
    static Eigen::Quaternion<typename Derived::Scalar> positify(const Eigen::QuaternionBase<Derived> &q)
    {
        return q;
    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qleft(const Eigen::QuaternionBase<Derived> &q)
    {
        Eigen::Quaternion<typename Derived::Scalar> qq = positify(q);
        Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
        ans(0, 0) = qq.w(), ans.template block<1, 3>(0, 1) = -qq.vec().transpose();
        ans.template block<3, 1>(1, 0) = qq.vec(), ans.template block<3, 3>(1, 1) = qq.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() + skewSymmetric(qq.vec());
        return ans;
    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qright(const Eigen::QuaternionBase<Derived> &p)
    {
        Eigen::Quaternion<typename Derived::Scalar> pp = positify(p);
        Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
        ans(0, 0) = pp.w(), ans.template block<1, 3>(0, 1) = -pp.vec().transpose();
        ans.template block<3, 1>(1, 0) = pp.vec(), ans.template block<3, 3>(1, 1) = pp.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() - skewSymmetric(pp.vec());
        return ans;
    }
};

void subSampleFrame(std::vector<point3D> &frame, double size_voxel);

void gridSampling(const std::vector<point3D> &frame, std::vector<point3D> &keypoints, double size_voxel_subsampling);

void distortFrame(std::vector<point3D> &points, Eigen::Quaterniond &q_begin, Eigen::Quaterniond &q_end, Eigen::Vector3d &t_begin, Eigen::Vector3d &t_end, Eigen::Matrix3d &R_imu_lidar, Eigen::Vector3d &t_imu_lidar);

void transformPoint(MotionCompensation compensation, point3D &point_temp, Eigen::Quaterniond &q_begin, Eigen::Quaterniond &q_end, Eigen::Vector3d &t_begin, Eigen::Vector3d &t_end, Eigen::Matrix3d &R_imu_lidar, Eigen::Vector3d &t_imu_lidar);

namespace std {
    template <typename T, typename... Args>
        std::unique_ptr<T> make_unique(Args&&... args) {
        return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
    }
}