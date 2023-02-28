#include "cloudProcessing.h"
#include "utility.h"

cloudProcessing::cloudProcessing()
{
	point_filter_num = 1;
	sweep_id = 0;
}

void cloudProcessing::setLidarType(int para)
{
	lidar_type = para;
}

void cloudProcessing::setNumScans(int para)
{
	N_SCANS = para;

	for(int i = 0; i < N_SCANS; i++){
		pcl::PointCloud<pcl::PointXYZINormal> v_cloud_temp;
		v_cloud_temp.clear();
		scan_cloud.push_back(v_cloud_temp);
	}

	assert(N_SCANS == scan_cloud.size());

	for(int i = 0; i < N_SCANS; i++){
		std::vector<extraElement> v_elem_temp;
		v_extra_elem.push_back(v_elem_temp);
	}

	assert(N_SCANS == v_extra_elem.size());
}

void cloudProcessing::setScanRate(int para)
{
	SCAN_RATE = para;
}

void cloudProcessing::setTimeUnit(int para)
{
	time_unit = para;

	switch (time_unit)
	{
	case SEC:
		time_unit_scale = 1.e3f;
		break;
	case MS:
		time_unit_scale = 1.f;
		break;
	case US:
		time_unit_scale = 1.e-3f;
		break;
	case NS:
		time_unit_scale = 1.e-6f;
		break;
	default:
		time_unit_scale = 1.f;
		break;
	}
}

void cloudProcessing::setBlind(double para)
{
	blind = para;
}

void cloudProcessing::setExtrinR(Eigen::Matrix3d &R)
{
	R_imu_lidar = R;
}

void cloudProcessing::setExtrinT(Eigen::Vector3d &t)
{
	t_imu_lidar = t;
}

void cloudProcessing::setPointFilterNum(int para)
{
	point_filter_num = para;
}

void cloudProcessing::setSweepCutNum(int para)
{
	sweep_cut_num = para;

	delta_cut_time = (1.0 / (double)SCAN_RATE * 1000.0) / sweep_cut_num;

	for(int i = 0; i < 2 * sweep_cut_num; i++){
		std::vector<point3D> v_point_temp;
		v_cut_sweep.push_back(v_point_temp);
	}
}

void cloudProcessing::process(const sensor_msgs::PointCloud2::ConstPtr &msg, std::vector<std::vector<point3D>> &v_cloud_out, std::vector<double> &v_dt_offset)
{
	switch (lidar_type)
	{
	case OUST64:
		ROS_ERROR("Only Velodyne LiDAR interface is supported currently.");
		break;

	case VELO16:
		velodyneHandler(msg, v_cloud_out, v_dt_offset);
		break;

	default:
		ROS_ERROR("Only Velodyne LiDAR interface is supported currently.");
		break;
	}

    if(debug_output)
    {
    	std::ofstream foutC(std::string(output_path + "/cutCloud.txt"), std::ios::app);

	    foutC.setf(std::ios::scientific, std::ios::floatfield);
	        foutC.precision(6);

	    foutC << std::fixed << msg->header.stamp.toSec() << " ";

    	int num = 0;

	    for(int i = 0; i < sweep_cut_num; i++)
	    {
	    	std::string sub_pcd_path(output_path + "/cut_sweep/" + std::to_string(sweep_id) + "_" + std::to_string(i) + std::string(".pcd"));

	    	foutC << v_cut_sweep[i + sweep_cut_num].size() << " ";
	    	num = num + v_cut_sweep[i + sweep_cut_num].size();

	    	pcl::PointCloud<pcl::PointXYZINormal>::Ptr p_cloud_temp;
	    	p_cloud_temp.reset(new pcl::PointCloud<pcl::PointXYZINormal>());

	    	point3DtoPCL(v_cut_sweep[i + sweep_cut_num], p_cloud_temp);

	    	saveCutCloud(sub_pcd_path, p_cloud_temp);
	    }

	    foutC << delta_cut_time << " ";
	    foutC << num;
	    foutC << std::endl;
	    foutC.close();
    }

    sweep_id++;
}

void cloudProcessing::oust64Handler(const sensor_msgs::PointCloud2::ConstPtr &msg, std::vector<std::vector<point3D>> &v_cloud_out)
{

}

void cloudProcessing::velodyneHandler(const sensor_msgs::PointCloud2::ConstPtr &msg, std::vector<std::vector<point3D>> &v_cloud_out, std::vector<double> &v_dt_offset)
{
	resetVector();

	pcl::PointCloud<velodyne_ros::Point> raw_cloud;
    pcl::fromROSMsg(*msg, raw_cloud);
    int size = raw_cloud.points.size();

    double dt_last_point;

    if(size == 0)
    {
    	v_cloud_out = v_cut_sweep;
    	v_dt_offset.resize(sweep_cut_num);

    	for (int i = 0; i < sweep_cut_num; i++)
    	{
    		v_dt_offset[i] = (i + 1) * delta_cut_time;
    	}
    	return;
    }

    if (raw_cloud.points[size - 1].time > 0)
     	given_offset_time = true;
    else
    	given_offset_time = false;

    if(given_offset_time)
    {
    	sort(raw_cloud.points.begin(), raw_cloud.points.end(), time_list_velodyne);
		while ((raw_cloud.points[size - 1].time >= 0.1)&&(time_unit == SEC))  //KAIST lidar's relative timestamp >0.1s 
		{
			size--;
			raw_cloud.points.pop_back();
		}
    	dt_last_point = raw_cloud.points.back().time * time_unit_scale;
    	delta_cut_time = dt_last_point / sweep_cut_num;
    }

    double omega = 0.361 * SCAN_RATE;

    std::vector<bool> is_first;
    is_first.resize(N_SCANS);
    fill(is_first.begin(), is_first.end(), true);

    std::vector<double> yaw_first_point;
    yaw_first_point.resize(N_SCANS);
    fill(yaw_first_point.begin(), yaw_first_point.end(), 0.0);

    std::vector<point3D> v_point_full;

	for(int i = 0; i < size; i++)
	{
		point3D point_temp;

        point_temp.raw_point = Eigen::Vector3d(raw_cloud.points[i].x, raw_cloud.points[i].y, raw_cloud.points[i].z);
        point_temp.point = point_temp.raw_point;
        point_temp.relative_time = raw_cloud.points[i].time * time_unit_scale;

        if(!given_offset_time)
		{
			int layer = raw_cloud.points[i].ring;
			double yaw_angle = atan2(point_temp.raw_point.y(), point_temp.raw_point.x()) * 57.2957;

			if (is_first[layer])
			{
				yaw_first_point[layer] = yaw_angle;
				is_first[layer] = false;
				point_temp.relative_time = 0.0;

				v_point_full.push_back(point_temp);

				continue;
			}

			if (yaw_angle <= yaw_first_point[layer])
			{
				point_temp.relative_time = (yaw_first_point[layer] - yaw_angle) / omega;
			}
			else
			{
				point_temp.relative_time = (yaw_first_point[layer] - yaw_angle + 360.0) / omega;
			}

			point_temp.timestamp = point_temp.relative_time / double(1000) + msg->header.stamp.toSec();
			v_point_full.push_back(point_temp);
		}

		if(given_offset_time && i % point_filter_num == 0)
		{
			if(point_temp.raw_point.x() * point_temp.raw_point.x() + point_temp.raw_point.y() * point_temp.raw_point.y()
				 + point_temp.raw_point.z() * point_temp.raw_point.z() > (blind * blind))
			{
				point_temp.timestamp = point_temp.relative_time / double(1000) + msg->header.stamp.toSec();
        		point_temp.alpha_time = point_temp.relative_time / dt_last_point;

				int id = int(point_temp.relative_time / delta_cut_time) + sweep_cut_num;

				id = id >= 2 * sweep_cut_num ? 2 * sweep_cut_num - 1 : id;

				v_cut_sweep[id].push_back(point_temp);
			}
		}
	}

	if(!given_offset_time)
	{
		assert(v_point_full.size() == size);

		sort(v_point_full.begin(), v_point_full.end(), time_list);
		dt_last_point = v_point_full.back().relative_time;
		delta_cut_time = dt_last_point / sweep_cut_num;

		for(int i = 0; i < size; i++)
		{
			if(i % point_filter_num == 0)
			{
				point3D point_temp = v_point_full[i];
	        	point_temp.alpha_time = (point_temp.relative_time / dt_last_point);

	        	if(point_temp.alpha_time > 1) point_temp.alpha_time = 1;
	        	if(point_temp.alpha_time < 0) point_temp.alpha_time = 0;

	        	int id = int(point_temp.relative_time / delta_cut_time) + sweep_cut_num;

				id = id >= 2 * sweep_cut_num ? 2 * sweep_cut_num - 1 : id;

				v_cut_sweep[id].push_back(point_temp);
			}
	    }
	}

	for(int i = 1; i <= sweep_cut_num - 1; i++)
    	v_dt_offset.push_back(delta_cut_time * i);

    v_dt_offset.push_back(dt_last_point);

    assert(v_dt_offset.size() == sweep_cut_num);

    v_cloud_out = v_cut_sweep;
}

void cloudProcessing::resetVector()
{
	while(v_cut_sweep.size() > sweep_cut_num)
	{
		std::vector<point3D>().swap(v_cut_sweep[0]);
		v_cut_sweep.erase(v_cut_sweep.begin());
	}

	while(v_cut_sweep.size() < 2 * sweep_cut_num)
	{
		std::vector<point3D> v_point_temp;
		v_cut_sweep.push_back(v_point_temp);
	}

	assert(v_cut_sweep.size() == 2 * sweep_cut_num);
}