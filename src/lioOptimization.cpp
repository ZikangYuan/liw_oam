#include "lioOptimization.h"

cloudFrame::cloudFrame(std::vector<point3D> &point_frame_, std::vector<point3D> &const_frame_, state *p_state_)
{
    point_frame.insert(point_frame.end(), point_frame_.begin(), point_frame_.end());
    const_frame.insert(const_frame.end(), const_frame_.begin(), const_frame_.end());

    p_state = p_state_;

    success = true;
}

cloudFrame::cloudFrame(cloudFrame *p_cloud_frame)
{
    time_sweep_begin = p_cloud_frame->time_sweep_begin;
    time_sweep_end = p_cloud_frame->time_sweep_end;
    time_frame_begin = p_cloud_frame->time_frame_begin;
    time_frame_end = p_cloud_frame->time_frame_end;

    id = p_cloud_frame->id;
    sub_id = p_cloud_frame->sub_id;
    frame_id = p_cloud_frame->frame_id;

    p_state = p_cloud_frame->p_state;

    point_frame.insert(point_frame.end(), p_cloud_frame->point_frame.begin(), p_cloud_frame->point_frame.end());
    const_frame.insert(const_frame.end(), p_cloud_frame->const_frame.begin(), p_cloud_frame->const_frame.end());

    offset_begin = p_cloud_frame->offset_begin;
    offset_end = p_cloud_frame->offset_end;
    dt_offset = p_cloud_frame->dt_offset;

    success = p_cloud_frame->success;
}

void cloudFrame::release()
{
    std::vector<point3D>().swap(point_frame);
    std::vector<point3D>().swap(const_frame);

    if(p_state != nullptr)
        p_state->release();

    delete p_state;

    p_state = nullptr;
}

estimationSummary::estimationSummary()
{

}

void estimationSummary::release()
{
    if(!state_frame) state_frame->release();

    std::vector<point3D>().swap(corrected_points);

    std::vector<point3D>().swap(all_corrected_points);

    std::vector<point3D>().swap(keypoints);
}

lioOptimization::lioOptimization()
{
	allocateMemory();

    readParameters();

    initialValue();

    pub_cloud_body = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_current", 2);
    pub_cloud_world = nh.advertise<sensor_msgs::PointCloud2>("/cloud_global_map", 2);
    pub_odom = nh.advertise<nav_msgs::Odometry>("/Odometry_after_opt", 5);
    pub_path = nh.advertise<nav_msgs::Path>("/path", 5);

    sub_cloud_ori = nh.subscribe<sensor_msgs::PointCloud2>(lidar_topic, 20000, &lioOptimization::standardCloudHandler, this);
    sub_imu_ori = nh.subscribe<sensor_msgs::Imu>(imu_topic, 50000, &lioOptimization::imuHandler, this);
    sub_wheel_ori = nh.subscribe<geometry_msgs::TwistStamped>(wheel_topic, 50000, &lioOptimization::wheelHandler, this);

    path.header.stamp = ros::Time::now();
    path.header.frame_id ="camera_init";
    points_world.reset(new pcl::PointCloud<pcl::PointXYZI>());

    //options.recordParameters();
}

void lioOptimization::readParameters()
{	
    int para_int;
    double para_double;
    bool para_bool;
    std::string str_temp;

    // common
    nh.param<std::string>("common/lidar_topic", lidar_topic, "/points_raw");
	nh.param<std::string>("common/imu_topic", imu_topic, "/imu_raw");
    nh.param<std::string>("common/wheel_topic", wheel_topic, "/wheel_topic");
    nh.param<int>("common/point_filter_num", para_int, 1);  cloud_pro->setPointFilterNum(para_int);
    nh.param<int>("common/sweep_cut_num", sweep_cut_num, 3); cloud_pro->setSweepCutNum(sweep_cut_num);
    nh.param<std::vector<double>>("common/gravity_acc", v_G, std::vector<double>());
    nh.param<bool>("debug_output", debug_output, false);
    nh.param<std::string>("output_path", output_path, "");

    // LiDAR parameter
    nh.param<int>("lidar_parameter/lidar_type", para_int, AVIA);  cloud_pro->setLidarType(para_int);
    nh.param<int>("lidar_parameter/N_SCANS", para_int, 16);  cloud_pro->setNumScans(para_int);
    nh.param<int>("lidar_parameter/SCAN_RATE", para_int, 10);  cloud_pro->setScanRate(para_int);
    nh.param<int>("lidar_parameter/time_unit", para_int, US);  cloud_pro->setTimeUnit(para_int);
    nh.param<double>("lidar_parameter/blind", para_double, 0.01);  cloud_pro->setBlind(para_double);
    nh.param<float>("lidar_parameter/det_range", det_range, 300.f);
    nh.param<double>("lidar_parameter/fov_degree", fov_deg, 180);

    // IMU parameter
    nh.param<double>("imu_parameter/acc_cov", para_double, 0.1);  imu_pro->setAccCov(para_double);
    nh.param<double>("imu_parameter/gyr_cov", para_double, 0.1);  imu_pro->setGyrCov(para_double);
    nh.param<double>("imu_parameter/b_acc_cov", para_double, 0.0001);  imu_pro->setBiasAccCov(para_double);
    nh.param<double>("imu_parameter/b_gyr_cov", para_double, 0.0001);  imu_pro->setBiasGyrCov(para_double);
    nh.param<double>("imu_parameter/vel_cov", para_double, 0.1);imu_pro->setVelCov(para_double);
    nh.param<bool>("imu_parameter/time_diff_enable", time_diff_enable, false);

    // extrinsic parameter
    nh.param<bool>("extrinsic_parameter/extrinsic_enable", extrin_enable, true);
    nh.param<std::vector<double>>("extrinsic_parameter/extrinsic_t", v_extrin_t, std::vector<double>());
    nh.param<std::vector<double>>("extrinsic_parameter/extrinsic_R", v_extrin_R, std::vector<double>());
    nh.param<std::vector<double>>("extrinsic_parameter/extrinsic_t_odom", v_extrin_t_odom, std::vector<double>());
    nh.param<std::vector<double>>("extrinsic_parameter/extrinsic_R_odom", v_extrin_R_odom, std::vector<double>());

    // new ct-icp
    nh.param<double>("odometry_options/init_voxel_size", options.init_voxel_size, 0.2);
    nh.param<double>("odometry_options/init_sample_voxel_size", options.init_sample_voxel_size, 1.0);
    nh.param<int>("odometry_options/init_num_frames", options.init_num_frames, 20);
    nh.param<double>("odometry_options/voxel_size", options.voxel_size, 0.5);
    nh.param<double>("odometry_options/sample_voxel_size", options.sample_voxel_size, 1.5);
    nh.param<double>("odometry_options/max_distance", options.max_distance, 100.0);
    nh.param<int>("odometry_options/max_num_points_in_voxel", options.max_num_points_in_voxel, 20);
    nh.param<double>("odometry_options/min_distance_points", options.min_distance_points, 0.1);
    nh.param<double>("odometry_options/distance_error_threshold", options.distance_error_threshold, 5.0);
    nh.param<int>("odometry_options/robust_minimal_level", options.robust_minimal_level, 0);
    nh.param<bool>("odometry_options/robust_registration", options.robust_registration, false);
    nh.param<double>("odometry_options/robust_full_voxel_threshold", options.robust_full_voxel_threshold, 0.7);
    nh.param<double>("odometry_options/robust_empty_voxel_threshold", options.robust_empty_voxel_threshold, 0.1);
    nh.param<double>("odometry_options/robust_neighborhood_min_dist", options.robust_neighborhood_min_dist, 0.10);
    nh.param<double>("odometry_options/robust_neighborhood_min_orientation", options.robust_neighborhood_min_orientation, 0.1);
    nh.param<double>("odometry_options/robust_relative_trans_threshold", options.robust_relative_trans_threshold, 1.0);
    nh.param<bool>("odometry_options/robust_fail_early", options.robust_fail_early, false);
    nh.param<int>("odometry_options/robust_num_attempts", options.robust_num_attempts, 6);
    nh.param<int>("odometry_options/robust_num_attempts_when_rotation", options.robust_num_attempts_when_rotation, 2);
    nh.param<int>("odometry_options/robust_max_voxel_neighborhood", options.robust_max_voxel_neighborhood, 3);
    nh.param<double>("odometry_options/robust_threshold_ego_orientation", options.robust_threshold_ego_orientation, 3);
    nh.param<double>("odometry_options/robust_threshold_relative_orientation", options.robust_threshold_relative_orientation, 3);

    nh.param<std::string>("odometry_options/method_system_init", str_temp, "MOTION_INIT");
    if(str_temp == "MOTION_INIT") options.method_system_init = MOTION_INIT;
    else if(str_temp == "STATIC_INIT") options.method_system_init = STATIC_INIT;
    else std::cout << "The `initialization_method` " << str_temp << " is not supported." << std::endl;

    nh.param<std::string>("odometry_options/motion_compensation", str_temp, "NONE");
    if(str_temp == "NONE") options.motion_compensation = NONE;
    else if(str_temp == "CONSTANT_VELOCITY") options.motion_compensation = CONSTANT_VELOCITY;
    else if(str_temp == "ITERATIVE") options.motion_compensation = ITERATIVE;
    else if(str_temp == "CONTINUOUS") options.motion_compensation = CONTINUOUS;
    else std::cout << "The `motion_compensation` " << str_temp << " is not supported." << std::endl;

    nh.param<std::string>("odometry_options/initialization", str_temp, "INIT_NONE");
    if(str_temp == "INIT_NONE") options.initialization = INIT_NONE;
    else if(str_temp == "INIT_CONSTANT_VELOCITY") options.initialization = INIT_CONSTANT_VELOCITY;
    else if(str_temp == "INIT_IMU") options.initialization = INIT_IMU;
    else std::cout << "The `state_initialization` " << str_temp << " is not supported." << std::endl;


    icpOptions optimize_options;
    nh.param<int>("icp_options/threshold_voxel_occupancy", options.optimize_options.threshold_voxel_occupancy, 1);
    nh.param<double>("icp_options/size_voxel_map", options.optimize_options.size_voxel_map, 1.0);
    nh.param<int>("icp_options/num_iters_icp", options.optimize_options.num_iters_icp, 5);
    nh.param<int>("icp_options/min_number_neighbors", options.optimize_options.min_number_neighbors, 20);
    nh.param<int>("icp_options/voxel_neighborhood", options.optimize_options.voxel_neighborhood, 1);
    nh.param<double>("icp_options/power_planarity", options.optimize_options.power_planarity, 2.0);
    nh.param<bool>("icp_options/estimate_normal_from_neighborhood", options.optimize_options.estimate_normal_from_neighborhood, true);
    nh.param<int>("icp_options/max_number_neighbors", options.optimize_options.max_number_neighbors, 20);
    nh.param<double>("icp_options/max_dist_to_plane_icp", options.optimize_options.max_dist_to_plane_icp, 0.3);
    nh.param<double>("icp_options/threshold_orientation_norm", options.optimize_options.threshold_orientation_norm, 0.0001);
    nh.param<double>("icp_options/threshold_translation_norm", options.optimize_options.threshold_translation_norm, 0.001);
    nh.param<bool>("icp_options/point_to_plane_with_distortion", options.optimize_options.point_to_plane_with_distortion, true);
    nh.param<int>("icp_options/max_num_residuals", options.optimize_options.max_num_residuals, -1);
    nh.param<int>("icp_options/min_num_residuals", options.optimize_options.min_num_residuals, 100);
    nh.param<int>("icp_options/num_closest_neighbors", options.optimize_options.num_closest_neighbors, 1);
    nh.param<double>("icp_options/beta_location_consistency", options.optimize_options.beta_location_consistency, 0.001);
    nh.param<double>("icp_options/beta_constant_velocity", options.optimize_options.beta_constant_velocity, 0.001);
    nh.param<double>("icp_options/beta_wheel_velocity", options.optimize_options.beta_wheel_velocity, 0.001);
    nh.param<double>("icp_options/beta_small_velocity", options.optimize_options.beta_small_velocity, 0.0);
    nh.param<double>("icp_options/beta_orientation_consistency", options.optimize_options.beta_orientation_consistency, 0.0);
    nh.param<double>("icp_options/weight_alpha", options.optimize_options.weight_alpha, 0.9);
    nh.param<double>("icp_options/weight_neighborhood", options.optimize_options.weight_neighborhood, 0.1);
    nh.param<int>("icp_options/ls_max_num_iters", options.optimize_options.ls_max_num_iters, 1);
    nh.param<int>("icp_options/ls_num_threads", options.optimize_options.ls_num_threads, 16);
    nh.param<double>("icp_options/ls_sigma", options.optimize_options.ls_sigma, 0.1);
    nh.param<double>("icp_options/ls_tolerant_min_threshold", options.optimize_options.ls_tolerant_min_threshold, 0.05);
    nh.param<bool>("icp_options/debug_print", options.optimize_options.debug_print, true);
    nh.param<bool>("icp_options/debug_viz", options.optimize_options.debug_viz, false);

    nh.param<std::string>("icp_options/distance", str_temp, "CT_POINT_TO_PLANE");
    if(str_temp == "POINT_TO_PLANE") options.optimize_options.distance = POINT_TO_PLANE;
    else if(str_temp == "CT_POINT_TO_PLANE") options.optimize_options.distance = CT_POINT_TO_PLANE;
    else std::cout << "The `icp_residual` " << str_temp << " is not supported." << std::endl;

    nh.param<std::string>("icp_options/weighting_scheme", str_temp, "ALL");
    if(str_temp == "PLANARITY") options.optimize_options.weighting_scheme = PLANARITY;
    else if(str_temp == "NEIGHBORHOOD") options.optimize_options.weighting_scheme = NEIGHBORHOOD;
    else if(str_temp == "ALL") options.optimize_options.weighting_scheme = ALL;
    else std::cout << "The `weighting_scheme` " << str_temp << " is not supported." << std::endl;

    nh.param<std::string>("icp_options/solver", str_temp, "LIO");
    if(str_temp == "LIO") options.optimize_options.solver = LIO;
    else if(str_temp == "LIDAR") options.optimize_options.solver = LIDAR;
    else if(str_temp == "LIW") options.optimize_options.solver = LIO, Odom_enble = true;
    else std::cout << "The `solve_method` " << str_temp << " is not supported." << std::endl;

    nh.param<std::string>("icp_options/loss_function", str_temp, "CAUCHY");
    if(str_temp == "CAUCHY") options.optimize_options.loss_function = CAUCHY;
    else if(str_temp == "STANDARD") options.optimize_options.loss_function = STANDARD;
    else if(str_temp == "HUBER") options.optimize_options.loss_function = HUBER;
    else if(str_temp == "TOLERANT") options.optimize_options.loss_function = TOLERANT;
    else if(str_temp == "TRUNCATED") options.optimize_options.loss_function = TRUNCATED;
    else std::cout << "The `loss_function` " << str_temp << " is not supported." << std::endl;

    nh.param<std::string>("icp_options/viz_mode", str_temp, "TIMESTAMP");
    if(str_temp == "TIMESTAMP") options.optimize_options.viz_mode = TIMESTAMP;
    else if(str_temp == "WEIGHT") options.optimize_options.viz_mode = WEIGHT;
    else if(str_temp == "NORMAL") options.optimize_options.viz_mode = NORMAL;
    else std::cout << "The `solve_method` " << str_temp << " is not supported." << std::endl;
    // new ct-icp
}

void lioOptimization::allocateMemory()
{
    cloud_pro = new cloudProcessing();
    imu_pro = new imuProcessing();

    down_cloud_body.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
    down_cloud_world.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
}

void lioOptimization::initialValue()
{
    laser_point_cov = 0.001;

    G = vec3FromArray(v_G);
    R_imu_lidar = mat33FromArray(v_extrin_R);
    t_imu_lidar = vec3FromArray(v_extrin_t);
    R_imu_odom = mat33FromArray(v_extrin_R_odom);
    t_imu_odom = vec3FromArray(v_extrin_t_odom);

    cloud_pro->setExtrinR(R_imu_lidar);
    cloud_pro->setExtrinT(t_imu_lidar);

    last_time_lidar = -1.0;
    last_time_imu = -1.0;
    last_time_frame = -1.0;
    current_time = -1.0;

    index_frame = 1;

    fov_deg = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
    
    ROS_INFO("odom_enble:....................................");
    if(Odom_enble == true)  std::cout << "odom_enble : true" << std::endl;
    else std::cout << "odom_enble : flase" << std::endl ;

    ImuFactor::t_io = t_imu_odom;
    ImuFactor::q_io = Eigen::Quaterniond(R_imu_odom);
    ImuFactor::odom_enble = Odom_enble;

    CTImuFactor::t_io = t_imu_odom;
    CTImuFactor::q_io = Eigen::Quaterniond(R_imu_odom);
    CTImuFactor::odom_enble = Odom_enble;

    imuIntegration::t_io = t_imu_odom;
    imuIntegration::q_io = Eigen::Quaterniond(R_imu_odom);
    imuIntegration::odom_enble = Odom_enble;
    imuProcessing::odom_enble = Odom_enble;

    LidarPlaneNormFactor::t_il = t_imu_lidar;
    LidarPlaneNormFactor::q_il = Eigen::Quaterniond(R_imu_lidar);
    LidarPlaneNormFactor::sqrt_info = sqrt(1 / laser_point_cov);

    CTLidarPlaneNormFactor::t_il = t_imu_lidar;
    CTLidarPlaneNormFactor::q_il = Eigen::Quaterniond(R_imu_lidar);
    CTLidarPlaneNormFactor::sqrt_info = sqrt(1 / laser_point_cov);

    registered_frames = 0;

    robust_num_consecutive_failures = 0;

    suspect_registration_error = false;

    options.optimize_options.init_num_frames = options.init_num_frames;

    switch(options.motion_compensation)
    {
        case NONE:
        case CONSTANT_VELOCITY:
            options.optimize_options.point_to_plane_with_distortion = false;
            options.optimize_options.distance = POINT_TO_PLANE;
            break;
        case ITERATIVE:
            options.optimize_options.point_to_plane_with_distortion = true;
            options.optimize_options.distance = POINT_TO_PLANE;
            break;
        case CONTINUOUS:
            options.optimize_options.point_to_plane_with_distortion = true;
            options.optimize_options.distance = CT_POINT_TO_PLANE;
            break;
    }
    next_robust_level = options.robust_minimal_level;
}

void lioOptimization::addPointToMap(voxelHashMap &map, const Eigen::Vector3d &point, double voxel_size, int max_num_points_in_voxel, double min_distance_points, int min_num_points, cloudFrame* p_frame)
{
    short kx = static_cast<short>(point[0] / voxel_size);
    short ky = static_cast<short>(point[1] / voxel_size);
    short kz = static_cast<short>(point[2] / voxel_size);

    voxelHashMap::iterator search = map.find(voxel(kx, ky, kz));

    if(search != map.end())
    {
        auto &voxel_block = (search.value());

        if(!voxel_block.IsFull()) {
            double sq_dist_min_to_points = 10 * voxel_size * voxel_size;
            for (int i(0); i < voxel_block.NumPoints(); ++i)
            {
                auto &_point = voxel_block.points[i];
                double sq_dist = (_point - point).squaredNorm();
                if (sq_dist < sq_dist_min_to_points)
                {
                    sq_dist_min_to_points = sq_dist;
                }
            }
            if(sq_dist_min_to_points > (min_distance_points * min_distance_points))
            {
                if(min_num_points <= 0 || voxel_block.NumPoints() >= min_num_points)
                {
                    voxel_block.AddPoint(point);
                    addPointToPcl(points_world, point, p_frame);
                }
            }
        }
    }
    else
    {
        if(min_num_points <= 0){
            voxelBlock block(max_num_points_in_voxel);
            block.AddPoint(point);
            map[voxel(kx, ky, kz)] = std::move(block);
        }

    }
}

void lioOptimization::addPointsToMap(voxelHashMap &map, cloudFrame* p_frame, double voxel_size, int max_num_points_in_voxel, double min_distance_points, int min_num_points)
{
    for (const auto &point: p_frame->point_frame)
    {
        addPointToMap(map, point.point, voxel_size, max_num_points_in_voxel, min_distance_points, min_num_points, p_frame);
    }
    publishCLoudWorld(pub_cloud_world, points_world, p_frame);
    points_world->clear();
}

void lioOptimization::removePointsFarFromLocation(voxelHashMap &map, const Eigen::Vector3d &location, double distance)
{
    std::vector<voxel> voxels_to_erase;
    for (auto &pair: map) {
        Eigen::Vector3d pt = pair.second.points[0];
        if ((pt - location).squaredNorm() > (distance * distance)) {
            voxels_to_erase.push_back(pair.first);
        }
    }
    for (auto &vox: voxels_to_erase)
        map.erase(vox);
}

size_t lioOptimization::mapSize(const voxelHashMap &map)
{
    size_t map_size(0);
    for (auto &itr_voxel_map: map) {
        map_size += (itr_voxel_map.second).NumPoints();
    }
    return map_size;
}

void lioOptimization::standardCloudHandler(const sensor_msgs::PointCloud2::ConstPtr &msg) 
{
    double sample_size = index_frame < options.init_num_frames ? options.init_voxel_size : options.voxel_size;

    assert(msg->header.stamp.toSec() > last_time_lidar);

    std::vector<std::vector<point3D>> v_cut_sweep;
    std::vector<double> v_dt_offset;

    cloud_pro->process(msg, v_cut_sweep, v_dt_offset);

    for(int i = 1; i < sweep_cut_num + 1; i++)
    {

        std::vector<std::vector<point3D>> v_buffer_temp;

        for(int j = i; j < i + sweep_cut_num; j++)
        {
            std::vector<point3D> frame(v_cut_sweep[j]);

            boost::mt19937_64 g;
            std::shuffle(frame.begin(), frame.end(), g);

            subSampleFrame(frame, sample_size);

            std::shuffle(frame.begin(), frame.end(), g);

            v_buffer_temp.push_back(frame);
        }

        assert(v_buffer_temp.size() == sweep_cut_num);
        lidar_buffer.push(v_buffer_temp);
        // ROS_INFO("LIDAR is received");
        time_buffer.push(std::make_pair(msg->header.stamp.toSec(), v_dt_offset[i - 1] / (double)1000.0));
    }

    assert(msg->header.stamp.toSec() > last_time_lidar);
    last_time_lidar = msg->header.stamp.toSec();
}

void lioOptimization::imuHandler(const sensor_msgs::Imu::ConstPtr &msg)
{
    sensor_msgs::Imu::Ptr msg_temp(new sensor_msgs::Imu(*msg));

    if (abs(time_diff) > 0.1 && time_diff_enable)
    {
        msg_temp->header.stamp = ros::Time().fromSec(time_diff + msg->header.stamp.toSec());
    }

    assert(msg_temp->header.stamp.toSec() > last_time_imu);

    imu_buffer.push(msg_temp);
    // ROS_INFO("IMU is received");
    assert(msg_temp->header.stamp.toSec() > last_time_imu);
    last_time_imu = msg_temp->header.stamp.toSec();
}

void lioOptimization::wheelHandler(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    geometry_msgs::TwistStamped::Ptr msg_temp(new geometry_msgs::TwistStamped(*msg));

    if (abs(time_diff) > 0.1 && time_diff_enable)
    {
        msg_temp->header.stamp = ros::Time().fromSec(time_diff + msg->header.stamp.toSec());
    }

    assert(msg_temp->header.stamp.toSec() > last_time_velo);

    wheel_buffer.push(msg_temp);

    assert(msg_temp->header.stamp.toSec() > last_time_velo);
    last_time_velo = msg_temp->header.stamp.toSec();

    double Lwheel =msg_temp->twist.linear.x;
    double Rwheel = msg_temp->twist.linear.y;
}

std::vector<std::pair<std::pair<std::vector<sensor_msgs::ImuConstPtr>, std::vector<std::vector<point3D>>>, std::pair<double, double>>>  lioOptimization::getMeasurements(std::vector<std::vector<geometry_msgs::TwistStamped::ConstPtr>> &vWheel_msg)
{
    std::vector<std::pair<std::pair<std::vector<sensor_msgs::ImuConstPtr>, std::vector<std::vector<point3D>>>, std::pair<double, double>>> measurements;
    while (true)
    {
        if (imu_buffer.size() < (60 / sweep_cut_num) || lidar_buffer.size() < 2 || time_buffer.size() < 2 || wheel_buffer.size() < 6/sweep_cut_num){
            return measurements;
        }

        if (!(imu_buffer.back()->header.stamp.toSec() > time_buffer.front().first + time_buffer.front().second))
        {
            return measurements;
        }

        if (!(wheel_buffer.back()->header.stamp.toSec() > time_buffer.front().first + time_buffer.front().second))
        {
            return measurements;
        }

        if (!(imu_buffer.front()->header.stamp.toSec() < time_buffer.front().first + time_buffer.front().second))
        {
            time_buffer.pop();

            for (int i = 0; i < lidar_buffer.front().size(); i++)
                std::vector<point3D>().swap(lidar_buffer.front()[i]);

            std::vector<std::vector<point3D>>().swap(lidar_buffer.front());
            assert(lidar_buffer.front().size() == 0);
            lidar_buffer.pop();

            continue;
        }

        if (!(wheel_buffer.front()->header.stamp.toSec() < time_buffer.front().first + time_buffer.front().second))
        {
            time_buffer.pop();

            for (int i = 0; i < lidar_buffer.front().size(); i++)
                std::vector<point3D>().swap(lidar_buffer.front()[i]);

            std::vector<std::vector<point3D>>().swap(lidar_buffer.front());
            assert(lidar_buffer.front().size() == 0);
            lidar_buffer.pop();

            continue;
        }

        double timestamp = time_buffer.front().first + time_buffer.front().second;
        double timestamp_begin = time_buffer.front().first;
        double timestamp_offset = time_buffer.front().second;
        time_buffer.pop();

        if (fabs(timestamp_begin - time_buffer.front().first) > 1e-5)
        {
            if (time_buffer.front().first - timestamp_begin - timestamp_offset > 1e-5)
            {
                timestamp_offset = time_buffer.front().first - timestamp_begin;
                timestamp = timestamp_begin + timestamp_offset;
            }
            else if (time_buffer.front().first - timestamp_begin - timestamp_offset < 1e-5)
            {
                timestamp_offset = time_buffer.front().first - timestamp_begin;
                timestamp = timestamp_begin + timestamp_offset;
            }
        }

        std::vector<std::vector<point3D>> v_cut_sweep = lidar_buffer.front();

        for (int i = 0; i < lidar_buffer.front().size(); i++)
            std::vector<point3D>().swap(lidar_buffer.front()[i]);
        std::vector<std::vector<point3D>>().swap(lidar_buffer.front());
        assert(lidar_buffer.front().size() == 0);
        lidar_buffer.pop();

        std::vector<sensor_msgs::ImuConstPtr> imu_measurements;
        while (imu_buffer.front()->header.stamp.toSec() < timestamp)
        {
            imu_measurements.emplace_back(imu_buffer.front());
            imu_buffer.pop();
        }

        imu_measurements.emplace_back(imu_buffer.front());

        std::vector<geometry_msgs::TwistStamped::ConstPtr> wheels;
        while (wheel_buffer.front()->header.stamp.toSec() < timestamp)
        {
            wheels.emplace_back(wheel_buffer.front());
            wheel_buffer.pop();
        }

        wheels.emplace_back(wheel_buffer.front());

        if (imu_measurements.empty()){
            std::cout << "no imu between two image" << std::endl;
            ROS_WARN("no imu between two image");
        }

        if (wheels.empty()){
            std::cout << "no wheel between two image" << std::endl;
            ROS_WARN("no wheel between two image");

        }

        measurements.emplace_back(std::make_pair(imu_measurements, v_cut_sweep), std::make_pair(timestamp_begin, timestamp_offset));
        vWheel_msg.push_back(wheels);
        break;
    }
    return measurements;
}

void lioOptimization::makePointTimestamp(std::vector<point3D> &sweep, double time_sweep_begin, double time_sweep_end)
{
    if(cloud_pro->isPointTimeEnable())
    {
        if (sweep_cut_num == 1) return;

        double delta_t = time_sweep_end - time_sweep_begin;

        for (int i = 0; i < sweep.size(); i++)
        {
            sweep[i].relative_time = sweep[i].timestamp - time_sweep_begin;
            sweep[i].alpha_time = sweep[i].relative_time / delta_t;
            sweep[i].relative_time = sweep[i].relative_time * 1000.0;
            if(sweep[i].alpha_time > 1.0) sweep[i].alpha_time = 1.0 - 1e-5;
        }
    }
    else
    {
        double delta_t = time_sweep_end - time_sweep_begin;

        std::vector<point3D>::iterator iter = sweep.begin();

        while (iter != sweep.end())
        {
            if((*iter).timestamp > time_sweep_end) iter = sweep.erase(iter);
            else if((*iter).timestamp < time_sweep_begin) iter = sweep.erase(iter);
            else
            {
                (*iter).relative_time = (*iter).timestamp - time_sweep_begin;
                (*iter).alpha_time = (*iter).relative_time / delta_t;
                (*iter).relative_time = (*iter).relative_time * 1000.0;
                iter++;
            }
        }
    }
}

cloudFrame* lioOptimization::buildFrame(std::vector<point3D> &const_frame, state *cur_state, double timestamp_begin, double timestamp_offset)
{
    std::vector<point3D> frame(const_frame);

    double offset_begin = sweep_cut_num == 1 ? 0 : index_frame % sweep_cut_num == 1 ? 0 : all_cloud_frame.back()->offset_end;
    double offset_end = timestamp_offset;

    double time_sweep_begin = index_frame <= sweep_cut_num ? timestamp_begin : index_frame % sweep_cut_num == 0 ? timestamp_begin : all_cloud_frame[all_cloud_frame.size() - sweep_cut_num + 1]->time_frame_begin;
    double time_frame_begin = sweep_cut_num == 1 ? timestamp_begin : index_frame % sweep_cut_num == 1 ? timestamp_begin : all_cloud_frame.back()->time_sweep_end;

    double dt_offset = 0;

    if(sweep_cut_num == 1)
    {   
        if(index_frame > sweep_cut_num)
            dt_offset -= time_frame_begin - all_cloud_frame.back()->time_sweep_end;
    }
    else
    {
        int i = all_cloud_frame.size() - 1;

        while(i >= 0 && all_cloud_frame[i]->sub_id != 0)
        {
            dt_offset += all_cloud_frame[i]->p_state->dt_buf.back();
            i--;
        }

        if(index_frame > sweep_cut_num)
            dt_offset -= (i == all_cloud_frame.size() - 1 ?  time_frame_begin : all_cloud_frame[i + 1]->time_frame_begin) - all_cloud_frame[i]->time_frame_end;
    }

    makePointTimestamp(frame, time_sweep_begin, timestamp_begin + timestamp_offset);

    if (index_frame <= sweep_cut_num + 1) {
        for (auto &point_temp: frame) {
            point_temp.alpha_time = 1.0;
        }
    }

    if (index_frame > sweep_cut_num + 1) {
        if (options.motion_compensation == CONSTANT_VELOCITY) {
            distortFrame(frame, cur_state->rotation_begin, cur_state->rotation, cur_state->translation_begin, cur_state->translation, R_imu_lidar, t_imu_lidar);
        }

        for (auto &point_temp: frame) {
            transformPoint(options.motion_compensation, point_temp, cur_state->rotation_begin, cur_state->rotation, cur_state->translation_begin, cur_state->translation, R_imu_lidar, t_imu_lidar);
        }
    }
    else
    {
        for (auto &point_temp: frame) {
            Eigen::Quaterniond q_identity = Eigen::Quaterniond::Identity();
            Eigen::Vector3d t_zero = Eigen::Vector3d::Zero();
            transformPoint(options.motion_compensation, point_temp, q_identity, q_identity, t_zero, t_zero, R_imu_lidar, t_imu_lidar);
        }
    }

    cloudFrame *p_frame = new cloudFrame(frame, const_frame, cur_state);
    p_frame->time_sweep_begin = time_sweep_begin;
    p_frame->time_sweep_end = timestamp_begin + timestamp_offset;
    p_frame->time_frame_begin = time_frame_begin;
    p_frame->time_frame_end = p_frame->time_sweep_end;
    p_frame->offset_begin = offset_begin;
    p_frame->offset_end = offset_end;
    p_frame->dt_offset = dt_offset;
    p_frame->id = all_cloud_frame.size();
    p_frame->sub_id = index_frame % sweep_cut_num;
    p_frame->frame_id = index_frame;

    all_cloud_frame.push_back(p_frame);

    return p_frame;
}

void lioOptimization::stateInitialization(state *cur_state)
{
    registered_frames++;

    if (index_frame <= sweep_cut_num + 1)
    {
        cur_state->rotation_begin = Eigen::Quaterniond::Identity();
        cur_state->translation_begin = Eigen::Vector3d::Zero();
        cur_state->rotation = Eigen::Quaterniond::Identity();
        cur_state->translation = Eigen::Vector3d::Zero();
    }
    else if (index_frame == sweep_cut_num + 2)
    {
        if (options.initialization == INIT_CONSTANT_VELOCITY)
        {
            Eigen::Quaterniond q_next_end = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation * 
                    all_cloud_frame[all_cloud_frame.size() - 2]->p_state->rotation.inverse() * all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation;

            Eigen::Vector3d t_next_end = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation + 
                                         all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation * 
                                         all_cloud_frame[all_cloud_frame.size() - 2]->p_state->rotation.inverse() * 
                                         (all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation - 
                                         all_cloud_frame[all_cloud_frame.size() - 2]->p_state->translation);

            cur_state->rotation_begin = all_cloud_frame[all_cloud_frame.size() - sweep_cut_num]->p_state->rotation;
            cur_state->translation_begin = all_cloud_frame[all_cloud_frame.size() - sweep_cut_num]->p_state->translation;
            cur_state->rotation = q_next_end;
            cur_state->translation = t_next_end;
        }
        else if (options.initialization == INIT_IMU)
        {
            if (initial_flag)
            {
                cur_state->rotation_begin = all_cloud_frame[all_cloud_frame.size() - sweep_cut_num]->p_state->rotation;
                cur_state->translation_begin = all_cloud_frame[all_cloud_frame.size() - sweep_cut_num]->p_state->translation;
            }
            else
            {
                Eigen::Quaterniond q_next_end = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation * 
                        all_cloud_frame[all_cloud_frame.size() - 2]->p_state->rotation.inverse() * all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation;

                Eigen::Vector3d t_next_end = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation + 
                                             all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation * 
                                             all_cloud_frame[all_cloud_frame.size() - 2]->p_state->rotation.inverse() * 
                                             (all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation - 
                                             all_cloud_frame[all_cloud_frame.size() - 2]->p_state->translation);

                cur_state->rotation_begin = all_cloud_frame[all_cloud_frame.size() - sweep_cut_num]->p_state->rotation;
                cur_state->translation_begin = all_cloud_frame[all_cloud_frame.size() - sweep_cut_num]->p_state->translation;
                cur_state->rotation = q_next_end;
                cur_state->translation = t_next_end;
            }
        }
        else
        {
            cur_state->rotation_begin = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation_begin;
            cur_state->translation_begin = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation_begin;
            cur_state->rotation = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation;
            cur_state->translation = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation;
        }
    }
    else
    {
        if (options.initialization == INIT_CONSTANT_VELOCITY)
        {
            if(options.motion_compensation == CONTINUOUS)
            {
                Eigen::Quaterniond q_next_begin = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation_begin * 
                        all_cloud_frame[all_cloud_frame.size() - 2]->p_state->rotation_begin.inverse() * all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation_begin;

                Eigen::Vector3d t_next_begin = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation_begin + 
                                               all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation_begin * 
                                               all_cloud_frame[all_cloud_frame.size() - 2]->p_state->rotation_begin.inverse() * 
                                               (all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation_begin - 
                                               all_cloud_frame[all_cloud_frame.size() - 2]->p_state->translation_begin);

                cur_state->rotation_begin = all_cloud_frame[all_cloud_frame.size() - sweep_cut_num]->p_state->rotation;
                cur_state->translation_begin = all_cloud_frame[all_cloud_frame.size() - sweep_cut_num]->p_state->translation;
            }
            else
            {
                cur_state->rotation_begin = all_cloud_frame[all_cloud_frame.size() - sweep_cut_num]->p_state->rotation;
                cur_state->translation_begin = all_cloud_frame[all_cloud_frame.size() - sweep_cut_num]->p_state->translation;
            }

            Eigen::Quaterniond q_next_end = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation * 
                    all_cloud_frame[all_cloud_frame.size() - 2]->p_state->rotation.inverse() * all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation;

            Eigen::Vector3d t_next_end = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation + 
                                         all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation * 
                                         all_cloud_frame[all_cloud_frame.size() - 2]->p_state->rotation.inverse() * 
                                         (all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation - 
                                         all_cloud_frame[all_cloud_frame.size() - 2]->p_state->translation);

            cur_state->rotation = q_next_end;
            cur_state->translation = t_next_end;
        }
        else if (options.initialization == INIT_IMU)
        {
            if (initial_flag)
            {
                cur_state->rotation_begin = all_cloud_frame[all_cloud_frame.size() - sweep_cut_num]->p_state->rotation;
                cur_state->translation_begin = all_cloud_frame[all_cloud_frame.size() - sweep_cut_num]->p_state->translation;
            }
            else
            {
                Eigen::Quaterniond q_next_end = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation * 
                        all_cloud_frame[all_cloud_frame.size() - 2]->p_state->rotation.inverse() * all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation;

                Eigen::Vector3d t_next_end = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation + 
                                             all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation * 
                                             all_cloud_frame[all_cloud_frame.size() - 2]->p_state->rotation.inverse() * 
                                             (all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation - 
                                             all_cloud_frame[all_cloud_frame.size() - 2]->p_state->translation);

                cur_state->rotation_begin = all_cloud_frame[all_cloud_frame.size() - sweep_cut_num]->p_state->rotation;
                cur_state->translation_begin = all_cloud_frame[all_cloud_frame.size() - sweep_cut_num]->p_state->translation;
                cur_state->rotation = q_next_end;
                cur_state->translation = t_next_end;
            }
        }
        else
        {
            cur_state->rotation_begin = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation_begin;
            cur_state->translation_begin = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation_begin;
            cur_state->rotation = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation;
            cur_state->translation = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation;
        }
    }
}

bool lioOptimization::assessRegistration(const cloudFrame *p_frame, estimationSummary &summary)
{

    bool success = summary.success;

    if(summary.robust_level == 0 && (summary.relative_orientation > options.robust_threshold_relative_orientation ||
         summary.ego_orientation > options.robust_threshold_ego_orientation))
    {
        if (summary.robust_level < options.robust_num_attempts_when_rotation) {
            summary.error_message = "Large rotations require at a robust_level of at least 1 (got:" +
                                    std::to_string(summary.robust_level) + ").";
            return false;
        }
    }

    if(summary.relative_distance > options.robust_relative_trans_threshold) {
        summary.error_message = "The relative distance is too important";
        return false;
    }

    bool do_neighbor_assessment = summary.distance_correction > 0.1;
    do_neighbor_assessment |= summary.relative_distance > options.robust_neighborhood_min_dist;
    do_neighbor_assessment |= summary.relative_orientation > options.robust_neighborhood_min_orientation;

    if(do_neighbor_assessment && registered_frames > options.init_num_frames)
    {
        if (options.robust_registration)
        {
            const double kSizeVoxelMap = options.optimize_options.size_voxel_map;
            voxel voxel_temp;
            double ratio_empty_voxel = 0;
            double ratio_half_full_voxel = 0;

            for (auto &point_temp: p_frame->point_frame) {
                voxel_temp = voxel::coordinates(point_temp.point, kSizeVoxelMap);
                if (voxel_map.find(voxel_temp) == voxel_map.end())
                    ratio_empty_voxel += 1;
                if (voxel_map.find(voxel_temp) != voxel_map.end() &&
                    voxel_map.at(voxel_temp).NumPoints() > options.max_num_points_in_voxel / 2) {
                    // Only count voxels which have at least
                    ratio_half_full_voxel += 1;
                }
            }

            ratio_empty_voxel /= p_frame->point_frame.size();
            ratio_half_full_voxel /= p_frame->point_frame.size();

            if (ratio_half_full_voxel < options.robust_full_voxel_threshold ||
                ratio_empty_voxel > options.robust_empty_voxel_threshold)
            {
                success = false;
                if (ratio_empty_voxel > options.robust_empty_voxel_threshold)
                    summary.error_message = "[Odometry::AssessRegistration] Ratio of empty voxels " +
                                            std::to_string(ratio_empty_voxel) + "above threshold.";
                else
                    summary.error_message = "[Odometry::AssessRegistration] Ratio of half full voxels " +
                                            std::to_string(ratio_half_full_voxel) + "below threshold.";

            }
        }
    }

    if (summary.relative_distance > options.distance_error_threshold)
    {
        return false;
    }

    return success;
}

estimationSummary lioOptimization::poseEstimation(cloudFrame *p_frame)
{
    auto start = std::chrono::steady_clock::now();

    icpOptions optimize_options = options.optimize_options;
    const double kSizeVoxelInitSample = options.voxel_size;

    const double kSizeVoxelMap = optimize_options.size_voxel_map;
    const double kMinDistancePoints = options.min_distance_points;
    const int kMaxNumPointsInVoxel = options.max_num_points_in_voxel;

    const state* initial_state = new state(p_frame->p_state, true);
    estimationSummary summary;
    summary.state_frame = new state(initial_state, true);
    state* previous_state = new state(initial_state, true);

    if(p_frame->frame_id > sweep_cut_num)
    {
        bool good_enough_registration = false;
        summary.number_of_attempts = 1;
        double sample_voxel_size = p_frame->frame_id < options.init_num_frames ? options.init_sample_voxel_size : options.sample_voxel_size;
        double min_voxel_size = std::min(options.init_voxel_size, options.voxel_size);

        auto increaseRobustnessLevel = [&]() {
            previous_state->release();
            previous_state = new state(summary.state_frame, true);
            
            p_frame->p_state = new state(initial_state, true);

            optimize_options.voxel_neighborhood = std::min(++optimize_options.voxel_neighborhood,
                                                         options.robust_max_voxel_neighborhood);
            optimize_options.ls_max_num_iters += 30;
            if (optimize_options.max_num_residuals > 0)
                optimize_options.max_num_residuals = optimize_options.max_num_residuals * 2;
            optimize_options.num_iters_icp = std::min(optimize_options.num_iters_icp + 20, 50);
            optimize_options.threshold_orientation_norm = std::max(
                    optimize_options.threshold_orientation_norm / 10, 1.e-5);
            optimize_options.threshold_translation_norm = std::max(
                    optimize_options.threshold_orientation_norm / 10, 1.e-4);
            sample_voxel_size = std::max(sample_voxel_size / 1.5, min_voxel_size);
            optimize_options.ls_sigma *= 1.2;
            optimize_options.max_dist_to_plane_icp *= 1.5;
        };

        summary.robust_level = 0;
        do {
            if(summary.robust_level < next_robust_level)
            {
                //increaseRobustnessLevel();
                continue;
            }

            auto start_ct_icp = std::chrono::steady_clock::now();
            optimize(p_frame, optimize_options, summary, sample_voxel_size);
            auto end_ct_icp = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed_icp = (end_ct_icp - start);

            if(p_frame->frame_id > sweep_cut_num)
            {
                summary.distance_correction = (p_frame->p_state->translation_begin - all_cloud_frame[p_frame->id - 1]->p_state->translation).norm();

                summary.relative_orientation = AngularDistance(all_cloud_frame[p_frame->id - 1]->p_state->rotation, p_frame->p_state->rotation);

                summary.ego_orientation = AngularDistance(summary.state_frame->rotation_begin, summary.state_frame->rotation);

            }

            summary.relative_distance = (p_frame->p_state->translation - p_frame->p_state->translation_begin).norm();

            good_enough_registration = assessRegistration(p_frame, summary);

            if(options.robust_fail_early)
                summary.success = good_enough_registration;

            if(!good_enough_registration)
            {
                if(options.robust_registration && summary.number_of_attempts < options.robust_num_attempts)
                {
                    double trans_distance = (previous_state->translation_begin - summary.state_frame->translation_begin).norm()
                                          + (previous_state->translation - summary.state_frame->translation).norm();

                    double rot_distance = ((previous_state->rotation_begin * summary.state_frame->rotation_begin.inverse()).toRotationMatrix() - Eigen::Matrix3d::Identity()).norm() 
                                        + ((previous_state->rotation * summary.state_frame->rotation.inverse()).toRotationMatrix() - Eigen::Matrix3d::Identity()).norm();

                    //increaseRobustnessLevel();
                    summary.robust_level++;
                    summary.number_of_attempts++;
                }
                else
                {
                    good_enough_registration = true;
                }
            }
        } while (!good_enough_registration);

        p_frame->success = summary.success;

        if(!summary.success)
        {
            return summary;
        }

        if(summary.number_of_attempts >= options.robust_num_attempts)
            robust_num_consecutive_failures++;
        else
            robust_num_consecutive_failures = 0;
    }

    bool add_points = true;

    if(options.robust_registration)
    {
        suspect_registration_error = summary.number_of_attempts >= options.robust_num_attempts;

        if (summary.ego_orientation > options.robust_threshold_ego_orientation ||
            summary.relative_orientation > options.robust_threshold_relative_orientation)
        {
            add_points = false;
        }

        if (suspect_registration_error) {
            add_points |= (robust_num_consecutive_failures > 5);
        }

        next_robust_level = add_points ? options.robust_minimal_level : options.robust_minimal_level + 1;
        if (!summary.success)
            next_robust_level = options.robust_minimal_level + 2;
        else {
            if (summary.relative_orientation > options.robust_threshold_relative_orientation ||
                summary.ego_orientation > options.robust_threshold_ego_orientation) {
                next_robust_level = options.robust_minimal_level + 1;
            }
            if (summary.number_of_attempts > 1) {
                next_robust_level = options.robust_minimal_level + 1;
            }
        }

    }

    if(p_frame->frame_id > sweep_cut_num && p_frame->sub_id != 0) add_points = false;

    if(add_points)
        addPointsToMap(voxel_map, p_frame, kSizeVoxelMap, kMaxNumPointsInVoxel, kMinDistancePoints);

    const double kMaxDistance = options.max_distance;
    const Eigen::Vector3d location = p_frame->p_state->translation;

    removePointsFarFromLocation(voxel_map, location, kMaxDistance);

    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;

    summary.corrected_points = p_frame->point_frame;
    summary.all_corrected_points = p_frame->const_frame;

    Eigen::Quaterniond q_begin = summary.state_frame->rotation_begin;
    Eigen::Quaterniond q_end = summary.state_frame->rotation;

    for (auto &point_temp: summary.all_corrected_points) {
        double alpha_time = point_temp.alpha_time;
        Eigen::Quaterniond slerp = q_begin.slerp(alpha_time, q_end).normalized();
        point_temp.point = slerp.toRotationMatrix() * point_temp.raw_point +
                     summary.state_frame->translation_begin * (1.0 - alpha_time) + alpha_time * summary.state_frame->translation;
    }

    return summary;
}

void lioOptimization::stateEstimation(std::vector<std::vector<point3D>> &v_cut_sweep, double timestamp_begin, double timestamp_offset)
{
    stateInitialization(imu_pro->current_state);

    std::vector<point3D> const_frame;

    for(int i = 0; i < v_cut_sweep.size(); i++)
        const_frame.insert(const_frame.end(), v_cut_sweep[i].begin(), v_cut_sweep[i].end());

    cloudFrame *p_frame = buildFrame(const_frame, imu_pro->current_state, timestamp_begin, timestamp_offset);

    estimationSummary summary = poseEstimation(p_frame);
    summary.release();

    if (options.optimize_options.solver == LIO && !initial_flag)
    {
        if(options.method_system_init == MOTION_INIT)
            motionInitialization();
        else if(options.method_system_init == STATIC_INIT)
            staticInitialization(p_frame);
    }

    std::cout << "after solution: " << std::endl;
    std::cout << "rotation_begin: " << p_frame->p_state->rotation_begin.x() << " " << p_frame->p_state->rotation_begin.y() << " " 
              << p_frame->p_state->rotation_begin.z() << " " << p_frame->p_state->rotation_begin.w() << std::endl;
    std::cout << "translation_begin: " << p_frame->p_state->translation_begin.x() << " " << p_frame->p_state->translation_begin.y() << " " << p_frame->p_state->translation_begin.z() << std::endl;

    std::cout << "rotation_end: " << p_frame->p_state->rotation.x() << " " << p_frame->p_state->rotation.y() << " " 
              << p_frame->p_state->rotation.z() << " " << p_frame->p_state->rotation.w() << std::endl;
    std::cout << "translation_end: " << p_frame->p_state->translation.x() << " " << p_frame->p_state->translation.y() << " " << p_frame->p_state->translation.z() << std::endl;

    imu_pro->last_state = imu_pro->current_state;
    imu_pro->current_state = new state(imu_pro->last_state, false);

    publish_odometry(pub_odom,p_frame);
    publish_path(pub_path,p_frame);   

    if(debug_output)
    {
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr p_cloud_temp;
        p_cloud_temp.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
        point3DtoPCL(p_frame->point_frame, p_cloud_temp);

        std::string pcd_path(output_path + "/cloud_frame/" + std::to_string(index_frame) + std::string(".pcd"));
        saveCutCloud(pcd_path, p_cloud_temp);
    }

    int num_remove = 0;

    if (initial_flag)
    {
        if (index_frame > sweep_cut_num && index_frame % sweep_cut_num == 0)
        {
            while (all_cloud_frame.size() > std::max(2, sweep_cut_num))
            {
                recordSinglePose(all_cloud_frame[0]);
                all_cloud_frame[0]->release();
                all_cloud_frame.erase(all_cloud_frame.begin());
                num_remove++;
            }
            assert(all_cloud_frame.size() == std::max(2, sweep_cut_num));
        }
    }
    else
    {
        while (all_cloud_frame.size() > options.num_for_initialization)
        {
            recordSinglePose(all_cloud_frame[0]);
            all_cloud_frame[0]->release();
            all_cloud_frame.erase(all_cloud_frame.begin());
            num_remove++;
        }
    }
    

    for(int i = 0; i < all_cloud_frame.size(); i++)
        all_cloud_frame[i]->id = all_cloud_frame[i]->id - num_remove;
}

void lioOptimization::recordSinglePose(cloudFrame *p_frame)
{
    std::ofstream foutC(std::string(output_path + "/pose.txt"), std::ios::app);

    foutC.setf(std::ios::scientific, std::ios::floatfield);
    foutC.precision(6);

    foutC << std::fixed << p_frame->time_sweep_end << " ";
    foutC << p_frame->p_state->translation.x() << " " << p_frame->p_state->translation.y() << " " << p_frame->p_state->translation.z() << " ";
    foutC << p_frame->p_state->rotation.x() << " " << p_frame->p_state->rotation.y() << " " << p_frame->p_state->rotation.z() << " " << p_frame->p_state->rotation.w();
    foutC << std::endl; 

    foutC.close();
}

void lioOptimization::set_posestamp(geometry_msgs::PoseStamped &body_pose_out,cloudFrame *p_frame)
{
    body_pose_out.pose.position.x = p_frame->p_state->translation.x();
    body_pose_out.pose.position.y = p_frame->p_state->translation.y();
    body_pose_out.pose.position.z = p_frame->p_state->translation.z();
    
    body_pose_out.pose.orientation.x = p_frame->p_state->rotation.x();
    body_pose_out.pose.orientation.y = p_frame->p_state->rotation.y();
    body_pose_out.pose.orientation.z = p_frame->p_state->rotation.z();
    body_pose_out.pose.orientation.w = p_frame->p_state->rotation.w();
}

void lioOptimization::publish_path(ros::Publisher pub_path,cloudFrame *p_frame)
{
    set_posestamp(msg_body_pose,p_frame);
    msg_body_pose.header.stamp = ros::Time().fromSec(p_frame->time_sweep_end);
    msg_body_pose.header.frame_id = "camera_init";

    static int i = 0;
    i++;
    if (i % 10 == 0) 
    {
        path.poses.push_back(msg_body_pose);
        pub_path.publish(path);
    }
}

void lioOptimization::publishCLoudWorld(ros::Publisher &pub_cloud_world, pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_points, cloudFrame* p_frame)
{
    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*pcl_points, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(p_frame->time_sweep_end);
    laserCloudmsg.header.frame_id = "camera_init";
    pub_cloud_world.publish(laserCloudmsg);
}

void lioOptimization::addPointToPcl(pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_points, const Eigen::Vector3d& point, cloudFrame *p_frame)
{
    pcl::PointXYZI cloudTemp;
    
    cloudTemp.x = point.x();
    cloudTemp.y = point.y();
    cloudTemp.z = point.z();
    cloudTemp.intensity = 50*(point.z()- p_frame->p_state->translation.z());
    pcl_points->points.push_back(cloudTemp);
}


void lioOptimization::publish_odometry(const ros::Publisher & pubOdomAftMapped, cloudFrame *p_frame)
{
    geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(p_frame->p_state->rotation.z(), -p_frame->p_state->rotation.x(), -p_frame->p_state->rotation.y());

    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = ros::Time().fromSec(p_frame->time_sweep_end);
    odomAftMapped.pose.pose.orientation.x = -geoQuat.y;
    odomAftMapped.pose.pose.orientation.y = -geoQuat.z;
    odomAftMapped.pose.pose.orientation.z = geoQuat.x;
    odomAftMapped.pose.pose.orientation.w = geoQuat.w;
    odomAftMapped.pose.pose.position.x = p_frame->p_state->translation.x();
    odomAftMapped.pose.pose.position.y = p_frame->p_state->translation.y();
    odomAftMapped.pose.pose.position.z = p_frame->p_state->translation.z();
    pubOdomAftMapped.publish(odomAftMapped);

    laserOdometryTrans.frame_id_ = "/camera_init";
    laserOdometryTrans.child_frame_id_ = "/laser_odom";
    laserOdometryTrans.stamp_ = ros::Time().fromSec(p_frame->time_sweep_end);;
    laserOdometryTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
    laserOdometryTrans.setOrigin(tf::Vector3(p_frame->p_state->translation.x(), p_frame->p_state->translation.y(), p_frame->p_state->translation.z()));
    tfBroadcaster.sendTransform(laserOdometryTrans);
}

void lioOptimization::run()
{
    std::vector<std::pair<std::pair<std::vector<sensor_msgs::ImuConstPtr>, std::vector<std::vector<point3D>>>, std::pair<double, double>>> measurements;
    std::vector<std::vector<geometry_msgs::TwistStamped::ConstPtr>> vWheel_msg;

    measurements = getMeasurements( vWheel_msg);

    if (measurements.size() == 0 || vWheel_msg.size() == 0) return;

    int numData = 0;
    for (auto &measurement : measurements)
    {
        auto start_lio_opt = std::chrono::steady_clock::now();

        auto v_cut_sweep = measurement.first.second;
        double time_frame = measurement.second.first + measurement.second.second;
        double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0, vLeft = 0, vRight = 0, speed = 0;
        int i = 0, numData = 0;

        for (auto &imu_msg : measurement.first.first)
        {   
            double time_imu = imu_msg->header.stamp.toSec();

            double wheel_t = vWheel_msg[numData][i]->header.stamp.toSec();

            if (i == 0)
            {
                vLeft = vWheel_msg[numData][i]->twist.linear.x;
                vRight = vWheel_msg[numData][i]->twist.linear.y;

                int n = 0;
                while (i + n + 1 < vWheel_msg[numData].size() && vWheel_msg[numData][i + n]->header.stamp.toSec() < time_imu)
                {
                    n++;
                }
                if (n > 0)
                    i = i + n - 1;
                else
                    i = 0;
            }
            else if (wheel_t < time_imu)
            {
                int n = 0;
                while (i + n + 1 < vWheel_msg[numData].size() && vWheel_msg[numData][i + n]->header.stamp.toSec() < time_imu)
                {
                    n++;
                }
                if (vWheel_msg[numData][i + n]->header.stamp.toSec() < time_imu)
                {
                    i = i + n;
                    vLeft = vWheel_msg[numData][i]->twist.linear.x;
                    vRight = vWheel_msg[numData][i]->twist.linear.y;
                }
                else if (vWheel_msg[numData][i + n]->header.stamp.toSec() >= time_imu)
                {
                    double dt_1 = time_imu - vWheel_msg[numData][i + n - 1]->header.stamp.toSec();
                    double dt_2 = vWheel_msg[numData][i + n]->header.stamp.toSec() - time_imu;
                    ROS_ASSERT(dt_1 >= 0);
                    ROS_ASSERT(dt_2 >= 0);
                    ROS_ASSERT(dt_1 + dt_2 > 0);
                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);
                    vLeft = w1 * vWheel_msg[numData][i + n - 1]->twist.linear.x + w2 * vWheel_msg[numData][i + n]->twist.linear.x;
                    vRight = w1 * vWheel_msg[numData][i + n - 1]->twist.linear.y + w2 * vWheel_msg[numData][i + n]->twist.linear.y;
                    i = i + n - 1;
                }
            }
            else if (time_imu >= wheel_t)
            {
                double dt_1 = time_imu - current_time;
                double dt_2 = vWheel_msg[numData][i]->header.stamp.toSec() - time_imu;
                ROS_ASSERT(dt_1 >= 0);
                ROS_ASSERT(dt_2 >= 0);
                ROS_ASSERT(dt_1 + dt_2 > 0);
                double w1 = dt_2 / (dt_1 + dt_2);
                double w2 = dt_1 / (dt_1 + dt_2);
                vLeft = w1 * vLeft + w2 * vWheel_msg[numData][i]->twist.linear.x;
                vRight = w1 * vRight + w2 * vWheel_msg[numData][i]->twist.linear.y;
            }
            if (time_imu <= time_frame)
            {
                if (current_time < 0)
                    current_time = measurement.second.first;
                double dt = time_imu - current_time;
                if (dt < -1e-6)
                    continue;
                assert(dt >= 0);
                current_time = time_imu;
                dx = imu_msg->linear_acceleration.x;
                dy = imu_msg->linear_acceleration.y;
                dz = imu_msg->linear_acceleration.z;
                rx = imu_msg->angular_velocity.x;
                ry = imu_msg->angular_velocity.y;
                rz = imu_msg->angular_velocity.z;
                speed = (vLeft + vRight) * 0.5;
		Eigen::Vector3d wheel_velocity = R_imu_odom * Eigen::Vector3d(speed, 0, 0);
                imu_pro->process(dt, Eigen::Vector3d(dx, dy, dz), Eigen::Vector3d(rx, ry, rz), wheel_velocity, time_imu);

                if (options.optimize_options.solver == LIO && !initial_flag)
                {
                    v_acc_static.push_back(Eigen::Vector3d(dx, dy, dz));
                    v_gyr_static.push_back(Eigen::Vector3d(rx, ry, rz));
                }
            }
            else
            {
                double dt_1 = time_frame - current_time;
                double dt_2 = time_imu - time_frame;
                current_time = time_frame;
                assert(dt_1 >= 0);
                assert(dt_2 >= 0);
                assert(dt_1 + dt_2 > 0);
                double w1 = dt_2 / (dt_1 + dt_2);
                double w2 = dt_1 / (dt_1 + dt_2);
                dx = w1 * dx + w2 * imu_msg->linear_acceleration.x;
                dy = w1 * dy + w2 * imu_msg->linear_acceleration.y;
                dz = w1 * dz + w2 * imu_msg->linear_acceleration.z;
                rx = w1 * rx + w2 * imu_msg->angular_velocity.x;
                ry = w1 * ry + w2 * imu_msg->angular_velocity.y;
                rz = w1 * rz + w2 * imu_msg->angular_velocity.z;
                speed = w1 * speed + w2 * (vLeft + vRight) * 0.5;
		Eigen::Vector3d wheel_velocity = R_imu_odom * Eigen::Vector3d(speed, 0, 0);
                imu_pro->process(dt_1, Eigen::Vector3d(dx, dy, dz), Eigen::Vector3d(rx, ry, rz), wheel_velocity, time_frame);
            }
        }
        std::cout <<  " velocity : " << speed << std::endl;
        numData++;
        stateEstimation(v_cut_sweep, measurement.second.first, measurement.second.second);
        
        last_time_frame = time_frame;
        index_frame++;

        for(int i = 0; i < measurement.first.second.size(); i++) std::vector<point3D>().swap(measurement.first.second[i]);
        std::vector<std::vector<point3D>>().swap(measurement.first.second);

        auto end_lio_opt = std::chrono::steady_clock::now();
        std::chrono::duration<double> per_scan_time = (end_lio_opt - start_lio_opt);
        std::cout << "per_scan_time: " << per_scan_time.count() << "s." << std::endl;
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "lio_optimization");
    ros::Time::init();
    
    lioOptimization LIO;

    ros::Rate rate(200);
    while (ros::ok())
    {
        ros::spinOnce();

        LIO.run();

        rate.sleep();
    }

    return 0;
}
