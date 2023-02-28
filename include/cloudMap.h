#pragma once
// c++
#include <iostream>
#include <math.h>
#include <thread>
#include <fstream>
#include <vector>
#include <queue>

// eigen 
#include <Eigen/Core>

// robin_map
#include <tsl/robin_map.h>

struct point3D {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3d raw_point;
    Eigen::Vector3d point;
    double alpha_time = 0.0;
    double relative_time = 0.0;
    double timestamp = 0.0;
    int index_frame = -1;

    point3D() = default;
};

struct voxel {

    voxel() = default;

    voxel(short x, short y, short z) : x(x), y(y), z(z) {}

    bool operator==(const voxel &vox) const { return x == vox.x && y == vox.y && z == vox.z; }

    inline bool operator<(const voxel &vox) const {
        return x < vox.x || (x == vox.x && y < vox.y) || (x == vox.x && y == vox.y && z < vox.z);
    }

    inline static voxel coordinates(const Eigen::Vector3d &point, double voxel_size) {
        return {short(point.x() / voxel_size),
                short(point.y() / voxel_size),
                short(point.z() / voxel_size)};
    }

    short x;
    short y;
    short z;
};

struct voxelBlock {

    explicit voxelBlock(int num_points_ = 20) : num_points(num_points_) { points.reserve(num_points_); }

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> points;

    bool IsFull() const { return num_points == points.size(); }

    void AddPoint(const Eigen::Vector3d &point) {
        assert(num_points > points.size());
        points.push_back(point);
    }

    inline int NumPoints() const { return points.size(); }

    inline int Capacity() { return num_points; }

private:
    int num_points;
};

typedef tsl::robin_map<voxel, voxelBlock> voxelHashMap;

namespace std {

    template<> struct hash<voxel> {
        std::size_t operator()(const voxel &vox) const
        {
            const size_t kP1 = 73856093;
            const size_t kP2 = 19349669;
            const size_t kP3 = 83492791;
            return vox.x * kP1 + vox.y * kP2 + vox.z * kP3;
        }
    };
}