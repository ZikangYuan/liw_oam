#pragma once
// c++
#include <iostream>

// utility
#include "utility.h"

class icpOptions {

public:
    // The threshold on the voxel occupancy
    // To be considered in the neighbor search, a voxel must have at least threshold_voxel_occupancy points
    int threshold_voxel_occupancy = 1;

    int init_num_frames = 20; // The number of frames defining the initialization of the map

    double size_voxel_map = 1.0; //Max Voxel : -32767 to 32767 then 32km map for SIZE_VOXEL_MAP = 1m

    int num_iters_icp = 5; // The Maximum number of ICP iterations performed

    int min_number_neighbors = 20;

    int voxel_neighborhood = 1; // Visits the (3 * voxel_neighborhood)^3 neighboring voxels

    double power_planarity = 2.0; // The power of planarity defined in the weighting scheme

    // Whether to estimate the normal of the key point or the closest neighbor
    bool estimate_normal_from_neighborhood = true;

    int max_number_neighbors = 20;

    double max_dist_to_plane_icp = 0.3; // The maximum distance point-to-plane (OLD Version of ICP)

    double threshold_orientation_norm = 0.0001; // Threshold on rotation (deg) for ICP's stopping criterion

    double threshold_translation_norm = 0.001; // Threshold on translation (deg) for ICP's stopping criterion

    bool point_to_plane_with_distortion = true; // Whether to distort the frames at each ICP iteration

    int max_num_residuals = -1; // The maximum number of keypoints used

    int min_num_residuals = 100; // Below this number, CT_ICP will crash

    IcpDistance distance = CT_POINT_TO_PLANE;

    int num_closest_neighbors = 1; // The number of closest neighbors considered as residuals

    // TODO : Add Trajectory Constraints Options
    double beta_location_consistency = 0.001; // Constraints on location

    double beta_constant_velocity = 0.001; // Constraint on velocity

    double beta_wheel_velocity = 0.001;

    double beta_small_velocity = 0.0; // Constraint on the relative motion

    double beta_orientation_consistency = 0.0; // Constraint on the orientation consistency

    WeightingScheme weighting_scheme = ALL;

    double weight_alpha = 0.9;

    double weight_neighborhood = 0.1;

    SolveMethod solver = LIO;

    LeastSquares loss_function = CAUCHY;

    int ls_max_num_iters = 1;

    int ls_num_threads = 16;

    double ls_sigma = 0.1; // The robust parameter (for Cauchy, Huber or truncated least square)

    double ls_tolerant_min_threshold = 0.05; // The Tolerant

    // Debug params
    bool debug_print = true; // Whether to output debug information to std::cout

    bool debug_viz = false; // Whether to pass the key points to the ExplorationEngine

    VizMode viz_mode = TIMESTAMP;

    void recordParameters();
};

class odometryOptions {

public:
    /* Parameters for initialization of the map */
    double init_voxel_size = 0.2;

    double init_sample_voxel_size = 1.0;

    int init_num_frames = 20; // The number of frames defining the initialization of the map

    int num_for_initialization = 10;

    double voxel_size = 0.5;

    double sample_voxel_size = 1.5;

    double max_distance = 100.0; // The threshold on the voxel size to remove points from the map

    int max_num_points_in_voxel = 20; // The maximum number of points in a voxel

    double min_distance_points = 0.1; // The minimal distance between points in the map

    double distance_error_threshold = 5.0; // The Ego-Motion Distance considered as an error

    // Whether to assess the quality of the registration,
    // And try a new registration with more conservative parameters in case of failure
    int robust_minimal_level = 0;
    bool robust_registration = false;
    double robust_full_voxel_threshold = 0.7;
    double robust_empty_voxel_threshold = 0.1;
    double robust_neighborhood_min_dist = 0.10; // The minimum relative distance to launch a robust neighborhood test
    double robust_neighborhood_min_orientation = 0.1; // The minimum relative orientation to launch a robust neighborhood
    // Threshold on the relative transform (all motion at 10Hz should be below this value)
    double robust_relative_trans_threshold = 1.0;
    bool robust_fail_early = false; // Stop iterations if the final assessment of the registration is unsucessful
    int robust_num_attempts = 6;
    int robust_num_attempts_when_rotation = 2;
    int robust_max_voxel_neighborhood = 3;
    double robust_threshold_ego_orientation = 3; // Angle in degrees
    double robust_threshold_relative_orientation = 3; // Angle in degrees

    icpOptions optimize_options;

    MotionCompensation motion_compensation = CONTINUOUS;

    StateInitialization initialization = INIT_CONSTANT_VELOCITY;

    LioInitialization method_system_init = MOTION_INIT;


    static odometryOptions defaultDrivingProfile();

    static odometryOptions defaultRobustOutdoorLowInertia();

    static odometryOptions robustDrivingProfile();

    void recordParameters();

};