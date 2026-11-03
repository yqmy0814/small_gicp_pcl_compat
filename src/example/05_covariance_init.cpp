// SPDX-FileCopyrightText: Copyright 2024 Kenji Koide
// SPDX-FileCopyrightText: Copyright 2026 yqmy0814
// SPDX-License-Identifier: MIT

/// @brief Example of the covariance initialization methods.
/// @note  The first two sections show how to select the covariance initialization, the last one compares the
///        preprocessing cost and the registration accuracy of the methods.
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include <small_gicp/ann/gaussian_voxelmap.hpp>
#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/benchmark/read_points.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/registration/reduction_omp.hpp>
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/registration/registration_helper.hpp>
#include <small_gicp/util/downsampling_omp.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>
#include <small_gicp/util/voxel_covariance.hpp>

using namespace small_gicp;

/// @brief Choose the covariance initialization through RegistrationSetting.
/// @note  The downsampling and the covariance estimation are performed inside align(), so the caller only selects the
///        method. The setting is shared by the Eigen-based interface and the PCL interface.
void example1(const std::vector<Eigen::Vector4f>& target, const std::vector<Eigen::Vector4f>& source) {
  RegistrationSetting setting;
  setting.type = RegistrationSetting::VGICP;
  setting.downsampling_resolution = 0.25;
  setting.num_threads = 4;

  // Per-point covariance estimation with k nearest neighbors (default)
  setting.cov_init = RegistrationSetting::POINT_KNN;
  setting.num_neighbors = 10;
  const auto result_knn = align(target, source, Eigen::Isometry3d::Identity(), setting);
  std::cout << "--- T_target_source (cov_init = POINT_KNN) ---" << std::endl << result_knn.T_target_source.matrix() << std::endl;

  // One covariance per voxel estimated from the points inside the voxel
  // The voxel size must be larger than the point spacing, otherwise the voxels contain too few points.
  setting.cov_init = RegistrationSetting::VOXEL_RAW;
  setting.cov_voxel_size = 0.5;
  const auto result_raw = align(target, source, Eigen::Isometry3d::Identity(), setting);
  std::cout << "--- T_target_source (cov_init = VOXEL_RAW) ---" << std::endl << result_raw.T_target_source.matrix() << std::endl;

  // One covariance per voxel estimated from the num_neighbors nearest voxel centroids
  // The covariance scale follows the voxel size and does not depend on the point density inside the voxel.
  setting.cov_init = RegistrationSetting::VOXEL_FILTER;
  setting.cov_num_neighbors = 8;
  const auto result_filter = align(target, source, Eigen::Isometry3d::Identity(), setting);
  std::cout << "--- T_target_source (cov_init = VOXEL_FILTER) ---" << std::endl << result_filter.T_target_source.matrix() << std::endl;
}

/// @brief Estimate the covariances explicitly, e.g. when the point clouds are reused for successive registrations.
/// @note  The estimation has to be done after the downsampling and before the voxelmap construction.
void example2(const std::vector<Eigen::Vector4f>& target_points, const std::vector<Eigen::Vector4f>& source_points) {
  const int num_threads = 4;
  const double cov_voxel_size = 0.5;

  // Downsample
  auto target = voxelgrid_sampling_omp(PointCloud(target_points), 0.25, num_threads);
  auto source = voxelgrid_sampling_omp(PointCloud(source_points), 0.25, num_threads);

  // Estimate the covariances. Use the *_normals_covariances variants when the registration type consumes normals
  // (PLANE_ICP / COLORED_ICP). The candidates are:
  //   estimate_covariances_omp(*target, num_neighbors, num_threads);                    // per-point kNN
  //   estimate_voxel_raw_covariances_omp(*target, cov_voxel_size, num_threads);         // voxel, from the raw points
  //   estimate_voxel_filter_covariances_omp(*target, cov_voxel_size, 8, num_threads);   // voxel, from the centroids
  estimate_voxel_filter_covariances_omp(*target, cov_voxel_size, 8, num_threads);
  estimate_voxel_filter_covariances_omp(*source, cov_voxel_size, 8, num_threads);

  // VGICP aggregates the target covariances into a voxelmap, and uses the source covariances as they are
  auto target_voxelmap = create_gaussian_voxelmap(*target, 1.0);

  Registration<GICPFactor, ParallelReductionOMP> registration;
  registration.reduction.num_threads = num_threads;
  registration.rejector.max_dist_sq = 1.0;

  const auto result = registration.align(*target_voxelmap, *source, *target_voxelmap, Eigen::Isometry3d::Identity());
  std::cout << "--- T_target_source (explicit estimation, voxel_filter) ---" << std::endl << result.T_target_source.matrix() << std::endl;
}

/// @brief Number of points that got the weak constraint (Identity) instead of a valid covariance.
size_t count_weak_covariances(const PointCloud& cloud) {
  size_t num_weak = 0;
  for (size_t i = 0; i < cloud.size(); i++) {
    const Eigen::Matrix4d& cov = cloud.cov(i);
    if (std::abs(cov(3, 3)) < 1e-12 && (cov.block<3, 3>(0, 0) - Eigen::Matrix3d::Identity()).norm() < 1e-12) {
      num_weak++;
    }
  }
  return num_weak;
}

/// @brief Benchmark setting. Every entry of an array is swept, so the output covers the product of them.
struct BenchmarkConfig {
  std::vector<double> noise_sigmas = {0.0};                    ///< Gaussian noise added to both clouds [m]
  std::vector<double> downsampling_resolutions = {0.0, 0.25};  ///< Downsample resolutions [m], 0 = no downsampling
  std::vector<bool> use_vgicp = {true, false};                 ///< Matching method (true = VGICP, false = GICP)
  std::vector<double> cov_voxel_sizes = {0.5};                 ///< Voxel sizes of the voxel-based covariance estimation [m]
  std::vector<int> cov_num_neighbors = {8};                    ///< Number of neighboring voxels for VOXEL_FILTER
  std::vector<int> point_knn_neighbors = {10};                 ///< Numbers of neighbors for the per-point estimation
  double voxel_resolution = 1.0;                               ///< Voxel resolution of VGICP [m]
  int num_threads = 4;                                         ///< Number of threads
};

/// @brief One covariance estimation method with its name for the output.
struct CovarianceMethod {
  std::string name;
  std::function<void(PointCloud&)> estimate;
};

/// @brief Build the list of the covariance estimation methods from the benchmark setting.
std::vector<CovarianceMethod> make_covariance_methods(const BenchmarkConfig& config) {
  const auto identity_init = [](PointCloud& cloud) {
    Eigen::Matrix4d cov = Eigen::Matrix4d::Zero();
    cov.block<3, 3>(0, 0) = 1e-3 * Eigen::Matrix3d::Identity();
    for (size_t i = 0; i < cloud.size(); i++) {
      cloud.cov(i) = cov;
    }
  };

  std::vector<CovarianceMethod> methods;
  methods.push_back({"identity (control)", identity_init});

  for (const double voxel_size : config.cov_voxel_sizes) {
    methods.push_back(
      {"voxel_raw " + std::to_string(voxel_size).substr(0, 4),
       [voxel_size, num_threads = config.num_threads](PointCloud& cloud) { estimate_voxel_raw_covariances_omp(cloud, voxel_size, num_threads); }});
  }

  for (const double voxel_size : config.cov_voxel_sizes) {
    for (const int num_neighbors : config.cov_num_neighbors) {
      methods.push_back(
        {"voxel_filter " + std::to_string(voxel_size).substr(0, 4) + " k=" + std::to_string(num_neighbors),
         [voxel_size, num_neighbors, num_threads = config.num_threads](PointCloud& cloud) {
           estimate_voxel_filter_covariances_omp(cloud, voxel_size, num_neighbors, num_threads);
         }});
    }
  }

  for (const int num_neighbors : config.point_knn_neighbors) {
    methods.push_back(
      {"point_kNN k=" + std::to_string(num_neighbors),
       [num_neighbors, num_threads = config.num_threads](PointCloud& cloud) { estimate_covariances_omp(cloud, num_neighbors, num_threads); }});
  }

  return methods;
}

/// @brief Add Gaussian noise to the point cloud.
void add_gaussian_noise(std::vector<Eigen::Vector4f>& points, double sigma, unsigned int seed) {
  std::mt19937 mt(seed);
  std::normal_distribution<double> nd(0.0, sigma);
  for (auto& pt : points) {
    pt.head<3>() += Eigen::Vector3f(nd(mt), nd(mt), nd(mt));
  }
}

/// @brief Compare the preprocessing cost and the registration accuracy of the covariance initialization methods.
/// @note  The point clouds are downsampled once per resolution, and every covariance method is evaluated on the same
///        downsampled clouds, so that the covariance initialization is the only difference between the rows.
/// @param config              Benchmark setting
/// @param target_points       Target points (raw)
/// @param source_points       Source points (raw)
/// @param T_target_source_gt  Ground truth of T_target_source
void benchmark_covariance_initialization(
  const BenchmarkConfig& config,
  const std::vector<Eigen::Vector4f>& target_points,
  const std::vector<Eigen::Vector4f>& source_points,
  const Eigen::Isometry3d& T_target_source_gt) {
  const auto now = [] { return std::chrono::steady_clock::now(); };
  const auto msec = [](const std::chrono::steady_clock::time_point& t0) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  };
  const auto median = [](std::vector<double> values) {
    std::sort(values.begin(), values.end());
    return values[values.size() / 2];
  };

  const std::vector<CovarianceMethod> methods = make_covariance_methods(config);

  const std::string separator(120, '-');

  printf(
    "=== covariance initialization benchmark ===\n"
    "columns:\n"
    "  noise   Gaussian noise std added to both clouds before the downsampling\n"
    "  filter  Downsampling resolution (0 = no downsampling)\n"
    "  match   Registration method (VGICP = voxelized GICP, GICP = point-wise GICP)\n"
    "  cov     Covariance estimation method and its parameters\n"
    "  cov[ms] Covariance estimation time of the target and source clouds (median of 3 runs)\n"
    "  reg[ms] Matching time, including the target-side structure construction (voxelmap or kdtree)\n"
    "  total   cov[ms] + reg[ms]\n"
    "  rot     Rotation error against the ground truth\n"
    "  trans   Translation error against the ground truth\n"
    "  iter    Number of optimization iterations\n"
    "  weak    Ratio of target points that got the weak constraint (Identity) instead of a valid covariance\n"
    "\n%s\n",
    separator.c_str());
  printf(
    "%-9s %-9s %-6s %-21s %7s %10s %10s %10s %10s %6s %8s\n",
    "noise[mm]",
    "filter[m]",
    "match",
    "covariance",
    "cov[ms]",
    "reg[ms]",
    "total[ms]",
    "rot[deg]",
    "trans[mm]",
    "iter",
    "weak[%]");

  for (const double noise_sigma : config.noise_sigmas) {
    auto target_noisy = target_points;
    auto source_noisy = source_points;
    if (noise_sigma > 0.0) {
      add_gaussian_noise(target_noisy, noise_sigma, 42);
      add_gaussian_noise(source_noisy, noise_sigma, 43);
    }

    for (const double downsampling_resolution : config.downsampling_resolutions) {
      printf("%s\n", separator.c_str());

      PointCloud target(target_noisy);
      PointCloud source(source_noisy);
      if (downsampling_resolution > 0.0) {
        target = *voxelgrid_sampling_omp(target, downsampling_resolution, config.num_threads);
        source = *voxelgrid_sampling_omp(source, downsampling_resolution, config.num_threads);
      }

      for (const bool use_vgicp : config.use_vgicp) {
        for (const auto& method : methods) {
          // Preprocessing is repeated so that the reported cost is not dominated by a single cache-cold run
          std::vector<double> cov_times;
          PointCloud target_estimated;
          PointCloud source_estimated;
          for (int i = 0; i < 3; i++) {
            PointCloud target_work = target;
            PointCloud source_work = source;

            const auto t_start = now();
            method.estimate(target_work);
            method.estimate(source_work);
            cov_times.emplace_back(msec(t_start));

            target_estimated = std::move(target_work);
            source_estimated = std::move(source_work);
          }

          const size_t num_weak = count_weak_covariances(target_estimated);

          Registration<GICPFactor, ParallelReductionOMP> registration;
          registration.reduction.num_threads = config.num_threads;
          registration.rejector.max_dist_sq = 1.0;

          // The target-side structure construction (voxelmap or kdtree) is included in the matching time
          const auto t_start = now();
          RegistrationResult result;
          if (use_vgicp) {
            auto target_voxelmap = create_gaussian_voxelmap(target_estimated, config.voxel_resolution);
            result = registration.align(*target_voxelmap, source_estimated, *target_voxelmap, Eigen::Isometry3d::Identity());
          } else {
            auto target_cloud = std::make_shared<PointCloud>(target_estimated);
            auto target_tree = std::make_shared<KdTree<PointCloud>>(target_cloud, KdTreeBuilderOMP(config.num_threads));
            result = registration.align(*target_cloud, source_estimated, *target_tree, Eigen::Isometry3d::Identity());
          }
          const double reg_time = msec(t_start);

          const Eigen::Isometry3d error = T_target_source_gt.inverse() * result.T_target_source;
          const double cov_time = median(cov_times);
          printf(
            "%-9.1f %-9.3f %-6s %-21s %7.2f %10.2f %10.2f %10.3f %10.3f %6d %8.1f\n",
            noise_sigma * 1000.0,
            downsampling_resolution,
            use_vgicp ? "VGICP" : "GICP",
            method.name.c_str(),
            cov_time,
            reg_time,
            cov_time + reg_time,
            Eigen::AngleAxisd(error.linear()).angle() * 180.0 / M_PI,
            error.translation().norm() * 1000.0,
            static_cast<int>(result.iterations),
            100.0 * static_cast<double>(num_weak) / static_cast<double>(target.size()));
        }
      }
    }
  }

  printf("%s\n", separator.c_str());
}

int main() {
  const auto target_points = read_ply("data/target.ply");
  const auto source_points = read_ply("data/source.ply");
  if (target_points.empty() || source_points.empty()) {
    std::cerr << "error: failed to read points from data/(target|source).ply" << std::endl;
    return 1;
  }

  std::ifstream ifs("data/T_target_source.txt");
  if (!ifs) {
    std::cerr << "error: failed to open data/T_target_source.txt" << std::endl;
    return 1;
  }
  Eigen::Matrix4d T_target_source_gt_matrix;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      ifs >> T_target_source_gt_matrix(i, j);
    }
  }
  const Eigen::Isometry3d T_target_source_gt(T_target_source_gt_matrix);

  example1(target_points, source_points);
  example2(target_points, source_points);

  printf(
    "\nground truth: rot %.3f deg, trans %.1f mm\n",
    Eigen::AngleAxisd(T_target_source_gt.linear()).angle() * 180.0 / M_PI,
    T_target_source_gt.translation().norm() * 1000.0);

  BenchmarkConfig config;
  benchmark_covariance_initialization(config, target_points, source_points, T_target_source_gt);

  return 0;
}
