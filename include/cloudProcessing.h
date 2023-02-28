#pragma once
// c++
#include <iostream>
#include <math.h>

// ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// eigen 
#include <Eigen/Core>

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include "cloudMap.h"

enum LID_TYPE{AVIA = 1, VELO16 = 2, OUST64 = 3};
enum TIME_UNIT{SEC = 0, MS = 1, US = 2, NS = 3};
enum Feature{Nor, Poss_Plane, Real_Plane, Edge_Jump, Edge_Plane, Wire, ZeroPoint};
enum Surround{Prev, Next};
enum Edge_jump{Nr_nor, Nr_zero, Nr_180, Nr_inf, Nr_blind};

struct extraElement
{
	double range;
	double distance;
	double angle[2];
	double intersect;
	Edge_jump edge_jump[2];
	Feature feature_type;

	extraElement()
	{
		range = 0;
		edge_jump[Prev] = Nr_nor;
		edge_jump[Next] = Nr_nor;
		feature_type = Nor;
		intersect = 2;
	}
};

namespace velodyne_ros {
	struct EIGEN_ALIGN16 Point {
	  PCL_ADD_POINT4D;
	  float intensity;
	  float time;
	  uint16_t ring;
	  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
}

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time, time)
    (uint16_t, ring, ring)
)

class cloudProcessing
{
private:

	int lidar_type;
	int N_SCANS;
	int SCAN_RATE;
	int time_unit;
	double time_unit_scale;
	double blind;

	Eigen::Matrix3d R_imu_lidar;
	Eigen::Vector3d t_imu_lidar;

	bool given_offset_time;

	int point_filter_num;
	int sweep_cut_num;

	int sweep_id;

	double delta_cut_time;

	std::vector<std::vector<point3D>> v_cut_sweep;
	std::vector<pcl::PointCloud<pcl::PointXYZINormal>> scan_cloud;
	std::vector<std::vector<extraElement>> v_extra_elem;

	// function
	void resetVector();
	void oust64Handler(const sensor_msgs::PointCloud2::ConstPtr &msg, std::vector<std::vector<point3D>> &v_cloud_out);
	void velodyneHandler(const sensor_msgs::PointCloud2::ConstPtr &msg, std::vector<std::vector<point3D>> &v_cloud_out, std::vector<double> &v_dt_offset);


public:

	cloudProcessing();

	void setLidarType(int para);
	void setNumScans(int para);
	void setScanRate(int para);
	void setTimeUnit(int para);
	void setBlind(double para);

	void setExtrinR(Eigen::Matrix3d &R);
	void setExtrinT(Eigen::Vector3d &t);

	void setUseFeature(bool para);
	void setPointFilterNum(int para);
	void setSweepCutNum(int para);

	int getLidarType() {return lidar_type;}
	int getSweepCutNum() {return sweep_cut_num;}

	bool isPointTimeEnable() {return given_offset_time;}

	void process(const sensor_msgs::PointCloud2::ConstPtr &msg, std::vector<std::vector<point3D>> &v_cloud_out, std::vector<double> &v_dt_offset);
};