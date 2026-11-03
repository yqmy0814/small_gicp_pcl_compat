// SPDX-FileCopyrightText: Copyright 2024 Kenji Koide
// SPDX-FileCopyrightText: Copyright 2026 yqmy0814
// SPDX-License-Identifier: MIT
#pragma once

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <limits>
#include <type_traits>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/points/traits.hpp>
#include <small_gicp/util/fast_floor.hpp>
#include <small_gicp/util/normal_estimation.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>
#include <small_gicp/util/sort_omp.hpp>

namespace small_gicp {

namespace detail {

/// @brief Setter used by the voxel-based estimators (writes normals as well only when requested).
template <bool WithNormals, typename PointCloud>
using VoxelSetter = std::conditional_t<WithNormals, NormalCovarianceSetter<PointCloud>, CovarianceSetter<PointCloud>>;

/// @brief Compute the packed voxel coords, and sort the points so that the points of the same voxel are contiguous.
/// @param cloud        [in] Point cloud
/// @param leaf_size    Voxel size
/// @param num_threads  Number of threads
/// @param coord_pt     [out] Sorted (packed voxel coord, point index) pairs
/// @param voxel_begin  [out] Beginning index of each voxel in coord_pt
template <typename PointCloud>
void group_points_by_voxel(
  const PointCloud& cloud,
  double leaf_size,
  int num_threads,
  std::vector<std::pair<std::uint64_t, size_t>>& coord_pt,
  std::vector<size_t>& voxel_begin) {
  const size_t num_points = traits::size(cloud);
  const double inv_leaf_size = 1.0 / leaf_size;

  constexpr std::uint64_t invalid_coord = std::numeric_limits<std::uint64_t>::max();
  constexpr int coord_bit_size = 21;
  constexpr size_t coord_bit_mask = (1 << 21) - 1;
  constexpr int coord_offset = 1 << (coord_bit_size - 1);

  coord_pt.resize(num_points);
#pragma omp parallel for num_threads(num_threads) schedule(guided, 32)
  for (std::int64_t i = 0; i < static_cast<std::int64_t>(num_points); i++) {
    const Eigen::Array4i coord = fast_floor(traits::point(cloud, i) * inv_leaf_size) + coord_offset;
    if ((coord < 0).any() || (coord > coord_bit_mask).any()) {
      std::cerr << "warning: voxel coord is out of range!!" << std::endl;
      coord_pt[i] = {invalid_coord, i};
      continue;
    }
    const std::uint64_t bits =                                                           //
      (static_cast<std::uint64_t>(coord[0] & coord_bit_mask) << (coord_bit_size * 0)) |  //
      (static_cast<std::uint64_t>(coord[1] & coord_bit_mask) << (coord_bit_size * 1)) |  //
      (static_cast<std::uint64_t>(coord[2] & coord_bit_mask) << (coord_bit_size * 2));
    coord_pt[i] = {bits, i};
  }

  quick_sort_omp(coord_pt.begin(), coord_pt.end(), [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; }, num_threads);

  voxel_begin.clear();
  voxel_begin.reserve(num_points / 8 + 1);
  for (size_t i = 0; i < num_points; i++) {
    if (i == 0 || coord_pt[i].first != coord_pt[i - 1].first) {
      voxel_begin.emplace_back(i);
    }
  }
}

/// @brief Estimation body shared by the covariance-only and the normal+covariance variants.
template <bool WithNormals, typename PointCloud>
void estimate_voxel_raw_impl(PointCloud& cloud, double leaf_size, int num_threads) {
  const size_t num_points = traits::size(cloud);
  if (num_points == 0) {
    return;
  }

  std::vector<std::pair<std::uint64_t, size_t>> coord_pt;
  std::vector<size_t> voxel_begin;
  group_points_by_voxel(cloud, leaf_size, num_threads, coord_pt, voxel_begin);

  constexpr std::uint64_t invalid_coord = std::numeric_limits<std::uint64_t>::max();
  using Setter = VoxelSetter<WithNormals, PointCloud>;

#pragma omp parallel for num_threads(num_threads) schedule(guided, 8)
  for (std::int64_t v = 0; v < static_cast<std::int64_t>(voxel_begin.size()); v++) {
    const size_t begin = voxel_begin[v];
    const size_t end = (v + 1 < static_cast<std::int64_t>(voxel_begin.size())) ? voxel_begin[v + 1] : num_points;
    const size_t num_voxel_points = end - begin;

    // Out of range points and voxels with too few points get the weak constraint used by estimate_covariances
    if (coord_pt[begin].first == invalid_coord || num_voxel_points < 5) {
      for (size_t i = begin; i < end; i++) {
        Setter::set_invalid(cloud, coord_pt[i].second);
      }
      continue;
    }

    // Accumulate the scatter matrix relative to the first point of the voxel to keep the numerical accuracy
    const Eigen::Vector3d reference = traits::point(cloud, coord_pt[begin].second).template head<3>();
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    Eigen::Matrix3d sum_cross = Eigen::Matrix3d::Zero();
    for (size_t i = begin; i < end; i++) {
      const Eigen::Vector3d pt = traits::point(cloud, coord_pt[i].second).template head<3>() - reference;
      sum += pt;
      sum_cross += pt * pt.transpose();
    }

    const Eigen::Vector3d mean = sum / static_cast<double>(num_voxel_points);
    const Eigen::Matrix3d cov = (sum_cross - mean * sum.transpose()) / static_cast<double>(num_voxel_points);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig;
    eig.computeDirect(cov);

    for (size_t i = begin; i < end; i++) {
      Setter::set(cloud, coord_pt[i].second, eig.eigenvectors());
    }
  }
}

/// @brief Estimation body shared by the covariance-only and the normal+covariance variants.
template <bool WithNormals, typename PointCloud>
void estimate_voxel_filter_impl(PointCloud& cloud, double leaf_size, int num_neighbors, int num_threads) {
  const size_t num_points = traits::size(cloud);
  if (num_points == 0) {
    return;
  }

  std::vector<std::pair<std::uint64_t, size_t>> coord_pt;
  std::vector<size_t> voxel_begin;
  group_points_by_voxel(cloud, leaf_size, num_threads, coord_pt, voxel_begin);

  constexpr std::uint64_t invalid_coord = std::numeric_limits<std::uint64_t>::max();

  // Build the voxel centroid cloud
  PointCloud centroid_cloud;
  traits::resize(centroid_cloud, voxel_begin.size());
  std::vector<size_t> centroid_of_voxel(voxel_begin.size(), std::numeric_limits<size_t>::max());
  size_t num_centroids = 0;
  for (size_t v = 0; v < voxel_begin.size(); v++) {
    const size_t begin = voxel_begin[v];
    const size_t end = (v + 1 < voxel_begin.size()) ? voxel_begin[v + 1] : num_points;
    if (coord_pt[begin].first == invalid_coord) {
      continue;
    }

    Eigen::Vector4d sum = Eigen::Vector4d::Zero();
    for (size_t i = begin; i < end; i++) {
      sum += traits::point(cloud, coord_pt[i].second);
    }
    centroid_of_voxel[v] = num_centroids++;
    traits::set_point(centroid_cloud, centroid_of_voxel[v], sum / sum.w());
  }
  traits::resize(centroid_cloud, num_centroids);

  if constexpr (WithNormals) {
    estimate_normals_covariances_omp(centroid_cloud, num_neighbors, num_threads);
  } else {
    estimate_covariances_omp(centroid_cloud, num_neighbors, num_threads);
  }

  using Setter = VoxelSetter<WithNormals, PointCloud>;
#pragma omp parallel for num_threads(num_threads) schedule(guided, 8)
  for (std::int64_t v = 0; v < static_cast<std::int64_t>(voxel_begin.size()); v++) {
    const size_t begin = voxel_begin[v];
    const size_t end = (v + 1 < static_cast<std::int64_t>(voxel_begin.size())) ? voxel_begin[v + 1] : num_points;
    if (centroid_of_voxel[v] == std::numeric_limits<size_t>::max()) {
      for (size_t i = begin; i < end; i++) {
        Setter::set_invalid(cloud, coord_pt[i].second);
      }
      continue;
    }

    const Eigen::Matrix4d cov = traits::cov(centroid_cloud, centroid_of_voxel[v]);
    if constexpr (WithNormals) {
      const Eigen::Vector4d normal = traits::normal(centroid_cloud, centroid_of_voxel[v]);
      for (size_t i = begin; i < end; i++) {
        const size_t idx = coord_pt[i].second;
        traits::set_cov(cloud, idx, cov);
        // Orient the normal towards the origin in the same way as NormalSetter (the sign depends on the point)
        traits::set_normal(cloud, idx, traits::point(cloud, idx).dot(normal) > 0 ? -normal : normal);
      }
    } else {
      for (size_t i = begin; i < end; i++) {
        traits::set_cov(cloud, coord_pt[i].second, cov);
      }
    }
  }
}

}  // namespace detail

/// @brief Estimate one covariance per voxel from the points inside the voxel and assign it to all points in that voxel.
/// @note  No nearest neighbor search is performed, so this is much faster than estimate_covariances. The covariance
///        scale is fully determined by leaf_size, and it does not adapt to the point density inside the voxel.
///        Voxels with fewer than 5 points are handled in the same way as estimate_covariances (weak constraints).
///        Voxel coordinates are limited to the 21 bit range (see voxelgrid_sampling_omp); points outside are ignored.
/// @param cloud       [in/out] Point cloud
/// @param leaf_size   Voxel size
/// @param num_threads Number of threads
template <typename PointCloud>
void estimate_voxel_raw_covariances_omp(PointCloud& cloud, double leaf_size, int num_threads) {
  detail::estimate_voxel_raw_impl<false>(cloud, leaf_size, num_threads);
}

/// @brief Estimate one covariance per voxel from the points inside the voxel and assign it to all points in that voxel.
/// @param cloud     [in/out] Point cloud
/// @param leaf_size Voxel size
template <typename PointCloud>
void estimate_voxel_raw_covariances(PointCloud& cloud, double leaf_size) {
  estimate_voxel_raw_covariances_omp(cloud, leaf_size, 1);
}

/// @brief Estimate one normal and one covariance per voxel from the points inside the voxel and assign them to all
///        points in that voxel.
/// @note  The point type must provide a normal field (e.g. pcl::PointNormalCovariance). As in estimate_normals, normals
///        are oriented towards the origin and the sign therefore depends on the point, not on the voxel. The normal is
///        only meaningful when the voxels contain enough points, i.e. when leaf_size is clearly larger than the point
///        spacing.
/// @param cloud       [in/out] Point cloud
/// @param leaf_size   Voxel size
/// @param num_threads Number of threads
template <typename PointCloud>
void estimate_voxel_raw_normals_covariances_omp(PointCloud& cloud, double leaf_size, int num_threads) {
  detail::estimate_voxel_raw_impl<true>(cloud, leaf_size, num_threads);
}

/// @brief Estimate one normal and one covariance per voxel from the points inside the voxel and assign them to all
///        points in that voxel.
/// @param cloud     [in/out] Point cloud
/// @param leaf_size Voxel size
template <typename PointCloud>
void estimate_voxel_raw_normals_covariances(PointCloud& cloud, double leaf_size) {
  estimate_voxel_raw_normals_covariances_omp(cloud, leaf_size, 1);
}

/// @brief Estimate covariances in voxel space: the covariance of each voxel is estimated by the num_neighbors nearest
///        voxel centroids and then assigned to all points in that voxel.
/// @note  Unlike estimate_voxel_raw_covariances, the neighborhood is searched on the voxel centroids, so the covariance
///        adapts to the surrounding geometry while the number of neighbors stays small. Voxels with too few centroid
///        neighbors are handled in the same way as estimate_covariances (weak constraints).
/// @param cloud         [in/out] Point cloud
/// @param leaf_size     Voxel size
/// @param num_neighbors Number of neighboring voxel centroids
/// @param num_threads   Number of threads
template <typename PointCloud>
void estimate_voxel_filter_covariances_omp(PointCloud& cloud, double leaf_size, int num_neighbors, int num_threads) {
  detail::estimate_voxel_filter_impl<false>(cloud, leaf_size, num_neighbors, num_threads);
}

/// @brief Estimate covariances in voxel space: the covariance of each voxel is estimated by the num_neighbors nearest
///        voxel centroids and then assigned to all points in that voxel.
/// @param cloud         [in/out] Point cloud
/// @param leaf_size     Voxel size
/// @param num_neighbors Number of neighboring voxel centroids
template <typename PointCloud>
void estimate_voxel_filter_covariances(PointCloud& cloud, double leaf_size, int num_neighbors = 8) {
  estimate_voxel_filter_covariances_omp(cloud, leaf_size, num_neighbors, 1);
}

/// @brief Estimate normals and covariances in voxel space: they are estimated by the num_neighbors nearest voxel
///        centroids and then assigned to all points in that voxel.
/// @note  The point type must provide a normal field (e.g. pcl::PointNormalCovariance).
/// @param cloud         [in/out] Point cloud
/// @param leaf_size     Voxel size
/// @param num_neighbors Number of neighboring voxel centroids
/// @param num_threads   Number of threads
template <typename PointCloud>
void estimate_voxel_filter_normals_covariances_omp(PointCloud& cloud, double leaf_size, int num_neighbors, int num_threads) {
  detail::estimate_voxel_filter_impl<true>(cloud, leaf_size, num_neighbors, num_threads);
}

/// @brief Estimate normals and covariances in voxel space: they are estimated by the num_neighbors nearest voxel
///        centroids and then assigned to all points in that voxel.
/// @param cloud         [in/out] Point cloud
/// @param leaf_size     Voxel size
/// @param num_neighbors Number of neighboring voxel centroids
template <typename PointCloud>
void estimate_voxel_filter_normals_covariances(PointCloud& cloud, double leaf_size, int num_neighbors = 8) {
  estimate_voxel_filter_normals_covariances_omp(cloud, leaf_size, num_neighbors, 1);
}

}  // namespace small_gicp
