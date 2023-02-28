#include "parameters.h"

odometryOptions odometryOptions::defaultDrivingProfile() {
    return odometryOptions{};
}

odometryOptions odometryOptions::robustDrivingProfile()
{
    odometryOptions default_options;

    default_options.voxel_size = 0.5;
    default_options.sample_voxel_size = 1.5;
    default_options.max_distance = 200.0;
    default_options.min_distance_points = 0.15;
    default_options.init_num_frames = 20;
    default_options.num_for_initialization = 10;
    default_options.max_num_points_in_voxel = 20;
    default_options.min_distance_points = 0.05;
    default_options.distance_error_threshold = 5.0;
    default_options.motion_compensation = CONTINUOUS;
    default_options.initialization = INIT_CONSTANT_VELOCITY;
    default_options.method_system_init = MOTION_INIT;

    default_options.robust_registration = true;
    default_options.robust_full_voxel_threshold = 0.5;
    default_options.robust_empty_voxel_threshold = 0.2;
    default_options.robust_num_attempts = 10;
    default_options.robust_max_voxel_neighborhood = 4;
    default_options.robust_threshold_relative_orientation = 5;
    default_options.robust_threshold_ego_orientation = 5;

    auto &optimize_options = default_options.optimize_options;
    optimize_options.debug_print = false;
    optimize_options.init_num_frames = 40;
    optimize_options.max_number_neighbors = 20;
    optimize_options.min_number_neighbors = 20;
    optimize_options.num_iters_icp = 15;
    optimize_options.max_dist_to_plane_icp = 0.5;
    optimize_options.threshold_orientation_norm = 0.1;
    optimize_options.threshold_orientation_norm = 0.01;
    optimize_options.point_to_plane_with_distortion = true;
    optimize_options.distance = CT_POINT_TO_PLANE;
    optimize_options.num_closest_neighbors = 1;
    optimize_options.beta_constant_velocity = 0.001;
    optimize_options.beta_location_consistency = 0.001;
    optimize_options.beta_small_velocity = 0.00;
    optimize_options.loss_function = CAUCHY;
    optimize_options.solver = LIO;
    optimize_options.ls_max_num_iters = 20;
    optimize_options.ls_num_threads = 8;
    optimize_options.ls_sigma = 0.2;
    optimize_options.ls_tolerant_min_threshold = 0.05;

    return default_options;
}

odometryOptions odometryOptions::defaultRobustOutdoorLowInertia()
{
    odometryOptions default_options;
    default_options.voxel_size = 0.3;
    default_options.sample_voxel_size = 1.5;
    default_options.min_distance_points = 0.1;
    default_options.max_distance = 200.0;
    default_options.init_num_frames = 20;
    default_options.num_for_initialization = 10;
    default_options.max_num_points_in_voxel = 20;
    default_options.distance_error_threshold = 5.0;
    default_options.motion_compensation = CONTINUOUS;
    default_options.initialization = INIT_NONE;
    default_options.method_system_init = MOTION_INIT;

    default_options.robust_registration = true;
    default_options.robust_full_voxel_threshold = 0.5;
    default_options.robust_empty_voxel_threshold = 0.1;
    default_options.robust_num_attempts = 3;
    default_options.robust_max_voxel_neighborhood = 4;
    default_options.robust_threshold_relative_orientation = 2;
    default_options.robust_threshold_ego_orientation = 2;

    auto &optimize_options = default_options.optimize_options;
    optimize_options.size_voxel_map = 0.8;
    optimize_options.num_iters_icp = 30;
    optimize_options.threshold_voxel_occupancy = 5;
    optimize_options.min_number_neighbors = 20;
    optimize_options.voxel_neighborhood = 1;

    optimize_options.init_num_frames = 20;
    optimize_options.max_number_neighbors = 20;
    optimize_options.min_number_neighbors = 20;
    optimize_options.max_dist_to_plane_icp = 0.5;
    optimize_options.threshold_orientation_norm = 0.1;
    optimize_options.threshold_orientation_norm = 0.01;
    optimize_options.point_to_plane_with_distortion = true;
    optimize_options.distance = CT_POINT_TO_PLANE;
    optimize_options.num_closest_neighbors = 1;
    optimize_options.beta_constant_velocity = 0.0;
    optimize_options.beta_location_consistency = 0.001;
    optimize_options.beta_small_velocity = 0.01;
    optimize_options.loss_function = CAUCHY;
    optimize_options.solver = LIO;
    optimize_options.ls_max_num_iters = 10;
    optimize_options.ls_num_threads = 8;
    optimize_options.ls_sigma = 0.2;
    optimize_options.ls_tolerant_min_threshold = 0.05;
    optimize_options.weight_neighborhood = 0.2;
    optimize_options.weight_alpha = 0.8;
    optimize_options.weighting_scheme = ALL;
    optimize_options.max_num_residuals = 600;
    optimize_options.min_num_residuals = 200;

    return default_options;
}

void odometryOptions::recordParameters()
{
	std::string str_temp;

	std::ofstream foutC(std::string(output_path + "/parameter_list.txt"), std::ios::app);

	foutC << "init_voxel_size: " << init_voxel_size << std::endl;
	foutC << "init_sample_voxel_size: " << init_sample_voxel_size << std::endl;
	foutC << "init_num_frames: " << init_num_frames << std::endl;
	foutC << "num_for_initialization: " << num_for_initialization << std::endl;
	foutC << "voxel_size: " << voxel_size << std::endl;
	foutC << "sample_voxel_size: " << sample_voxel_size << std::endl;
	foutC << "max_distance: " << max_distance << std::endl;
	foutC << "max_num_points_in_voxel: " << max_num_points_in_voxel << std::endl;
	foutC << "min_distance_points: " << min_distance_points << std::endl;
	foutC << "distance_error_threshold: " << distance_error_threshold << std::endl;
	foutC << "robust_minimal_level: " << robust_minimal_level << std::endl;
	str_temp = robust_registration ? "true" : "false";
	foutC << "robust_registration: " << str_temp << std::endl;
	foutC << "robust_full_voxel_threshold: " << robust_full_voxel_threshold << std::endl;
	foutC << "robust_empty_voxel_threshold: " << robust_empty_voxel_threshold << std::endl;
	foutC << "robust_neighborhood_min_dist: " << robust_neighborhood_min_dist << std::endl;
	foutC << "robust_neighborhood_min_orientation: " << robust_neighborhood_min_orientation << std::endl;
	foutC << "robust_relative_trans_threshold: " << robust_relative_trans_threshold << std::endl;
	str_temp = robust_fail_early ? "true" : "false";
	foutC << "robust_fail_early: " << str_temp << std::endl;
	foutC << "robust_num_attempts: " << robust_num_attempts << std::endl;
	foutC << "robust_num_attempts_when_rotation: " << robust_num_attempts_when_rotation << std::endl;
	foutC << "robust_max_voxel_neighborhood: " << robust_max_voxel_neighborhood << std::endl;
	foutC << "robust_threshold_ego_orientation: " << robust_threshold_ego_orientation << std::endl;
	foutC << "robust_threshold_relative_orientation: " << robust_threshold_relative_orientation << std::endl;
	switch(motion_compensation)
	{
	case 0:
		str_temp = "NONE";
		break;
	case 1:
		str_temp = "CONSTANT_VELOCITY";
		break;
	case 2:
		str_temp = "ITERATIVE";
		break;
	case 3:
		str_temp = "CONTINUOUS";
		break;
	default:
		break;
	}
	foutC << "motion_compensation: " << str_temp << std::endl;
	switch(method_system_init)
	{
	case 0:
		str_temp = "MOTION_INIT";
		break;
	case 1:
		str_temp = "STATIC_INIT";
		break;
	default:
		break;
	}
	foutC << "method_system_init: " << str_temp << std::endl;
	switch(initialization)
	{
	case 0:
		str_temp = "INIT_NONE";
		break;
	case 1:
		str_temp = "INIT_CONSTANT_VELOCITY";
		break;
	case 2:
		str_temp = "INIT_IMU";
		break;
	default:
		break;
	}
	foutC << "initialization: " << str_temp << std::endl;
    
    foutC.close();

    optimize_options.recordParameters();
}

void icpOptions::recordParameters()
{
	std::string str_temp;

	std::ofstream foutC(std::string(output_path + "/parameter_list.txt"), std::ios::app);

	foutC << "threshold_voxel_occupancy: " << threshold_voxel_occupancy << std::endl;
	foutC << "init_num_frames: " << init_num_frames << std::endl;
	foutC << "size_voxel_map: " << size_voxel_map << std::endl;
	foutC << "num_iters_icp: " << num_iters_icp << std::endl;
	foutC << "min_number_neighbors: " << min_number_neighbors << std::endl;
	foutC << "voxel_neighborhood: " << voxel_neighborhood << std::endl;
	foutC << "power_planarity: " << power_planarity << std::endl;
	str_temp = estimate_normal_from_neighborhood ? "true" : "false";
	foutC << "estimate_normal_from_neighborhood: " << str_temp << std::endl;
	foutC << "max_number_neighbors: " << max_number_neighbors << std::endl;
	foutC << "max_dist_to_plane_icp: " << max_dist_to_plane_icp << std::endl;
	foutC << "threshold_orientation_norm: " << threshold_orientation_norm << std::endl;
	foutC << "threshold_translation_norm: " << threshold_translation_norm << std::endl;
	str_temp = point_to_plane_with_distortion ? "true" : "false";
	foutC << "point_to_plane_with_distortion: " << str_temp << std::endl;
	foutC << "max_num_residuals: " << max_num_residuals << std::endl;
	foutC << "min_num_residuals: " << min_num_residuals << std::endl;
	switch(distance)
	{
	case 0:
		str_temp = "POINT_TO_PLANE";
		break;
	case 1:
		str_temp = "CT_POINT_TO_PLANE";
		break;
	default:
		break;
	}
	foutC << "distance: " << str_temp << std::endl;
	foutC << "num_closest_neighbors: " << num_closest_neighbors << std::endl;
	foutC << "beta_location_consistency: " << beta_location_consistency << std::endl;
	foutC << "beta_constant_velocity: " << beta_constant_velocity << std::endl;
	foutC << "beta_small_velocity: " << beta_small_velocity << std::endl;
	foutC << "beta_orientation_consistency: " << beta_orientation_consistency << std::endl;
	switch(weighting_scheme)
	{
	case 0:
		str_temp = "PLANARITY";
		break;
	case 1:
		str_temp = "NEIGHBORHOOD";
		break;
	case 2:
		str_temp = "ALL";
		break;
	default:
		break;
	}
	foutC << "weighting_scheme: " << str_temp << std::endl;
	foutC << "weight_alpha: " << weight_alpha << std::endl;
	foutC << "weight_neighborhood: " << weight_neighborhood << std::endl;
	switch(solver)
	{
	case 0:
		str_temp = "LIO";
		break;
	case 1:
		str_temp = "LIDAR";
		break;
	default:
		break;
	}
	foutC << "solver: " << str_temp << std::endl;
	switch(loss_function)
	{
	case 0:
		str_temp = "STANDARD";
		break;
	case 1:
		str_temp = "CAUCHY";
		break;
	case 2:
		str_temp = "HUBER";
		break;
	case 3:
		str_temp = "TOLERANT";
		break;
	case 4:
		str_temp = "TRUNCATED";
		break;
	default:
		break;
	}
	foutC << "loss_function: " << str_temp << std::endl;
	foutC << "ls_max_num_iters: " << ls_max_num_iters << std::endl;
	foutC << "ls_num_threads: " << ls_num_threads << std::endl;
	foutC << "ls_sigma: " << ls_sigma << std::endl;
	foutC << "ls_tolerant_min_threshold: " << ls_tolerant_min_threshold << std::endl;
	str_temp = debug_print ? "true" : "false";
	foutC << "debug_print: " << str_temp << std::endl;
	str_temp = debug_viz ? "true" : "false";
	foutC << "debug_viz: " << str_temp << std::endl;
	switch(viz_mode)
	{
	case 0:
		str_temp = "TIMESTAMP";
		break;
	case 1:
		str_temp = "WEIGHT";
		break;
	case 2:
		str_temp = "NORMAL";
		break;
	default:
		break;
	}
	foutC << "viz_mode: " << str_temp << std::endl;

	foutC.close();
}