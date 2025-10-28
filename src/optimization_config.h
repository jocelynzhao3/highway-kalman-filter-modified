#ifndef OPTIMIZATION_CONFIG_H_
#define OPTIMIZATION_CONFIG_H_

// Enable checking for dynamic allocations during development
#ifndef EIGEN_NO_MALLOC
#define EIGEN_NO_MALLOC
#endif

// Enable explicit vectorization
#ifndef EIGEN_VECTORIZE
#define EIGEN_VECTORIZE
#endif

// Align all fixed-size vectorizable Eigen objects
#ifndef EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#define EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif

#include <Eigen/Dense>
#include <Eigen/StdVector>

// Fixed-size type definitions for the UKF
namespace OptimizedTypes {

    // State dimensions
    static constexpr int n_x = 5;       // px, py, v, yaw, yawd
    static constexpr int n_aug = 7;     // Augmented state dimension
    static constexpr int n_sig = 2 * n_aug + 1; // Number of sigma points
    static constexpr int n_z_radar = 3; // Radar measurement dimension
    static constexpr int n_z_lidar = 2; // Lidar measurement dimension

    // Fixed-size Eigen types
    using StateVector = Eigen::Matrix<double, n_x, 1>;
    using AugmentedVector = Eigen::Matrix<double, n_aug, 1>;
    using StateCovMatrix = Eigen::Matrix<double, n_x, n_x>;
    using AugmentedCovMatrix = Eigen::Matrix<double, n_aug, n_aug>;
    using SigmaPointMatrix = Eigen::Matrix<double, n_x, n_sig>;
    using AugmentedSigmaPointMatrix = Eigen::Matrix<double, n_aug, n_sig>;
    using WeightVector = Eigen::Matrix<double, n_sig, 1>;

    using RadarMeasurementVector = Eigen::Matrix<double, n_z_radar, 1>;
    using LidarMeasurementVector = Eigen::Matrix<double, n_z_lidar, 1>;
    using RadarMeasurementMatrix = Eigen::Matrix<double, n_z_radar, n_sig>;
    using LidarMeasurementMatrix= Eigen::Matrix<double, n_z_lidar, n_sig>;

    // Aligned STL containers for Eigen types
    template<typename T>
    using AlignedVector = std::vector<T, Eigen::aligned_allocator<T>>;
}

#endif /* OPTIMIZATION_CONFIG_H_ */
