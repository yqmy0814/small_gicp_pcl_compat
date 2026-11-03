// SPDX-FileCopyrightText: Copyright 2024 Kenji Koide
// SPDX-FileCopyrightText: Copyright 2026 yqmy0814
// SPDX-License-Identifier: MIT
#pragma once

// Aggregated header for using small_gicp with PCL point clouds. It provides the PCL point types,
// the traits specialization, the point cloud conversion helpers, and the registration interface.

#include <vector>

#include <Eigen/Geometry>

#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/util/downsampling.hpp>
#include <small_gicp/util/downsampling_omp.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>

#include <small_gicp/factors/icp_factor.hpp>
#include <small_gicp/factors/plane_icp_factor.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/factors/colored_icp_factor.hpp>

#include <small_gicp/registration/reduction_omp.hpp>
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/registration/registration_helper.hpp>

#include <small_gicp/pcl/pcl_point.hpp>
#include <small_gicp/pcl/pcl_point_traits.hpp>

namespace small_gicp {

/// @brief Proxy class to access PCL point cloud with external covariance matrices.
template <typename PointT>
struct PointCloudProxy {
  PointCloudProxy(const pcl::PointCloud<PointT>& cloud, std::vector<Eigen::Matrix4d>& covs) : cloud(cloud), covs(covs) {}

  const pcl::PointCloud<PointT>& cloud;  ///< Point cloud
  std::vector<Eigen::Matrix4d>& covs;    ///< Covariance matrices
};

namespace traits {
template <typename PointT>
struct Traits<PointCloudProxy<PointT>> {
  using Points = PointCloudProxy<PointT>;

  static size_t size(const Points& points) { return points.cloud.size(); }

  static bool has_points(const Points& points) { return !points.cloud.points.empty(); }
  static bool has_covs(const Points& points) { return !points.covs.empty(); }

  static const Eigen::Vector4d point(const Points& points, size_t i) { return points.cloud.at(i).getVector4fMap().template cast<double>(); }
  static const Eigen::Matrix4d& cov(const Points& points, size_t i) { return points.covs[i]; }

  static void resize(Points& points, size_t n) { points.covs.resize(n); }
  static void set_cov(Points& points, size_t i, const Eigen::Matrix4d& cov) { points.covs[i] = cov; }
};

}  // namespace traits

/// @brief Convert a PCL-like point cloud into a vector of Eigen points.
/// @note  It only requires that the input cloud has a `points` member that is a vector of points
///        with `x`, `y`, and `z` members, which keeps the conversion independent of the PCL version.
/// @param cloud  Input point cloud (e.g., pcl::PointCloud<pcl::PointXYZ>)
/// @return       Points as a vector of Eigen::Vector4d (x, y, z, 1)
template <typename PointCloudT>
std::vector<Eigen::Vector4d> toEigenVector(const PointCloudT& cloud) {
  std::vector<Eigen::Vector4d> points;
  points.reserve(cloud.points.size());
  for (const auto& p : cloud.points) {
    points.emplace_back(p.x, p.y, p.z, 1.0);
  }
  return points;
}

/// @brief Convert a vector of Eigen points into a PCL-like point cloud.
/// @note  Only the point coordinates are written back. Other fields (intensity,
///        color, ...) are left untouched.
/// @param points  Points as a vector of Eigen::Vector4d
/// @param cloud   Output point cloud (e.g., pcl::PointCloud<pcl::PointXYZI>)
template <typename PointCloudT>
void fromEigenVector(const std::vector<Eigen::Vector4d>& points, PointCloudT& cloud) {
  cloud.resize(points.size());
  for (size_t i = 0; i < points.size(); i++) {
    cloud.points[i].x = static_cast<float>(points[i].x());
    cloud.points[i].y = static_cast<float>(points[i].y());
    cloud.points[i].z = static_cast<float>(points[i].z());
  }
}

/// @brief Write back point coordinates and covariances into a PCL-like point cloud.
/// @note  The output point type must provide a writable 4x4 covariance member
///        accessor (e.g., pcl::PointCovariance::getCovariance4fMap()).
/// @param points  Points as a vector of Eigen::Vector4d
/// @param covs    Covariances as a vector of Eigen::Matrix4d
/// @param cloud   Output point cloud
template <typename PointCloudT>
void fromEigenVector(const std::vector<Eigen::Vector4d>& points, const std::vector<Eigen::Matrix4d>& covs, PointCloudT& cloud) {
  cloud.resize(points.size());
  for (size_t i = 0; i < points.size(); i++) {
    auto& p = cloud.points[i];
    p.x = static_cast<float>(points[i].x());
    p.y = static_cast<float>(points[i].y());
    p.z = static_cast<float>(points[i].z());
    if (i < covs.size()) {
      p.getCovariance4fMap() = covs[i].cast<float>();
    }
  }
}

/// @brief Align PCL point clouds
/// @note  Input point clouds are downsampled and their normals and covariances are estimated internally
///        according to the given setting, so no preprocessing is required on the caller side.
/// @param target     Target point cloud
/// @param source     Source point cloud
/// @param init_T     Initial guess of T_target_source
/// @param setting    Registration setting
/// @return           Registration result
template <typename TargetPointT, typename SourcePointT>
RegistrationResult align(
  const pcl::PointCloud<TargetPointT>& target,
  const pcl::PointCloud<SourcePointT>& source,
  const Eigen::Isometry3d& init_T = Eigen::Isometry3d::Identity(),
  const RegistrationSetting& setting = RegistrationSetting()) {
  return align(toEigenVector(target), toEigenVector(source), init_T, setting);
}

}  // namespace small_gicp
