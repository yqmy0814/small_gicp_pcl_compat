#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <map>
#include <random>
#include <vector>

#include <small_gicp/pcl/pcl.hpp>
#include <small_gicp/util/voxel_covariance.hpp>

using namespace small_gicp;

namespace {

// Generate points on a slightly noisy plane z = 0
PointCloud make_plane_cloud(size_t num_points, double extent = 1.0) {
  std::mt19937 mt(42);
  std::uniform_real_distribution<> ud(-extent, extent);
  std::normal_distribution<> nd(0.0, 1e-3);

  PointCloud cloud;
  cloud.resize(num_points);
  for (size_t i = 0; i < num_points; i++) {
    cloud.point(i) << 3.0 + ud(mt), -2.0 + ud(mt), nd(mt), 1.0;
  }
  return cloud;
}

}  // namespace

// Every point in the same voxel must share one covariance, for both voxel-based estimators
TEST(VoxelCovarianceTest, SameVoxelSameCovariance) {
  const PointCloud cloud = make_plane_cloud(2000);
  const double leaf_size = 0.5;

  PointCloud raw = cloud;
  PointCloud filter = cloud;
  estimate_voxel_raw_covariances(raw, leaf_size);
  estimate_voxel_filter_covariances(filter, leaf_size, 8);

  for (const auto* estimated : {&raw, &filter}) {
    std::map<std::array<int, 3>, Eigen::Matrix4d> cov_of_voxel;
    size_t num_compared = 0;
    for (size_t i = 0; i < estimated->size(); i++) {
      const Eigen::Array3i coord = (estimated->point(i).head<3>() / leaf_size).array().floor().cast<int>();
      const std::array<int, 3> key = {coord[0], coord[1], coord[2]};
      const auto found = cov_of_voxel.find(key);
      if (found == cov_of_voxel.end()) {
        cov_of_voxel.emplace(key, estimated->cov(i));
      } else {
        EXPECT_LT((found->second - estimated->cov(i)).norm(), 1e-12);
        num_compared++;
      }
    }
    EXPECT_GT(cov_of_voxel.size(), 20);
    EXPECT_GT(num_compared, 100);
  }
}

// The estimated covariance must follow the plane model of the library (eigenvalues 1e-3, 1, 1 with the plane normal)
TEST(VoxelCovarianceTest, PlaneModel) {
  const PointCloud cloud = make_plane_cloud(2000);

  PointCloud raw = cloud;
  estimate_voxel_raw_covariances(raw, 0.5);

  size_t num_checked = 0;
  for (size_t i = 0; i < raw.size(); i++) {
    const Eigen::Matrix3d cov = raw.cov(i).block<3, 3>(0, 0);
    if (cov.norm() < 1e-6) {
      continue;
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig;
    eig.computeDirect(cov);
    EXPECT_NEAR(eig.eigenvalues()[0], 1e-3, 1e-6);
    EXPECT_NEAR(eig.eigenvalues()[1], 1.0, 1e-6);
    EXPECT_NEAR(eig.eigenvalues()[2], 1.0, 1e-6);
    EXPECT_NEAR(std::abs(eig.eigenvectors()(2, 0)), 1.0, 1e-2);
    num_checked++;
  }
  EXPECT_GT(num_checked, 100);
}

// Voxels with fewer than 5 points must get the weak constraint used by estimate_covariances
TEST(VoxelCovarianceTest, SparseVoxel) {
  PointCloud cloud;
  cloud.resize(4);
  for (size_t i = 0; i < 4; i++) {
    cloud.point(i) << 100.0 + 0.1 * i, 100.0, 100.0, 1.0;
  }

  PointCloud raw = cloud;
  estimate_voxel_raw_covariances(raw, 1.0);

  for (size_t i = 0; i < raw.size(); i++) {
    EXPECT_LT((raw.cov(i).block<3, 3>(0, 0) - Eigen::Matrix3d::Identity()).norm(), 1e-12);
    EXPECT_LT(std::abs(raw.cov(i)(3, 3)), 1e-12);
  }
}

// Empty point clouds must be handled without any access
TEST(VoxelCovarianceTest, EmptyCloud) {
  PointCloud cloud;
  estimate_voxel_raw_covariances(cloud, 0.5);
  estimate_voxel_raw_covariances_omp(cloud, 0.5, 4);
  estimate_voxel_filter_covariances(cloud, 0.5, 8);
  estimate_voxel_filter_covariances_omp(cloud, 0.5, 8, 4);
  EXPECT_TRUE(cloud.empty());
}

// Multi-threaded and single-threaded estimation must give the same result
TEST(VoxelCovarianceTest, SerialEqualsOmp) {
  const PointCloud cloud = make_plane_cloud(5000);

  PointCloud raw_serial = cloud;
  PointCloud raw_omp = cloud;
  estimate_voxel_raw_covariances(raw_serial, 0.5);
  estimate_voxel_raw_covariances_omp(raw_omp, 0.5, 4);

  PointCloud filter_serial = cloud;
  PointCloud filter_omp = cloud;
  estimate_voxel_filter_covariances(filter_serial, 0.5, 8);
  estimate_voxel_filter_covariances_omp(filter_omp, 0.5, 8, 4);

  for (size_t i = 0; i < cloud.size(); i++) {
    EXPECT_LT((raw_serial.cov(i) - raw_omp.cov(i)).norm(), 1e-12);
    EXPECT_LT((filter_serial.cov(i) - filter_omp.cov(i)).norm(), 1e-12);
  }
}

// The estimators must work on PCL point clouds through the traits specialization (with normals)
TEST(VoxelCovarianceTest, PclPointCloud) {
  const PointCloud cloud = make_plane_cloud(1000);

  pcl::PointCloud<pcl::PointCovariance> pcl_cloud;
  pcl_cloud.resize(cloud.size());
  for (size_t i = 0; i < cloud.size(); i++) {
    pcl_cloud.at(i).getVector4fMap() = cloud.point(i).cast<float>();
  }

  estimate_voxel_raw_covariances(pcl_cloud, 0.5);
  estimate_voxel_filter_covariances(pcl_cloud, 0.5, 8);

  EXPECT_EQ(pcl_cloud.size(), cloud.size());
  for (size_t i = 0; i < pcl_cloud.size(); i++) {
    EXPECT_TRUE(pcl_cloud.at(i).cov.allFinite());
  }
}

// Point types without a normal field (e.g. pcl::PointCovariance) must still work, normals are simply skipped
TEST(VoxelCovarianceTest, PclCovarianceOnlyPointCloud) {
  const PointCloud cloud = make_plane_cloud(1000);

  pcl::PointCloud<pcl::PointCovariance> pcl_cloud;
  pcl_cloud.resize(cloud.size());
  for (size_t i = 0; i < cloud.size(); i++) {
    pcl_cloud.at(i).getVector4fMap() = cloud.point(i).cast<float>();
  }

  estimate_voxel_raw_covariances(pcl_cloud, 0.5);
  estimate_voxel_filter_covariances(pcl_cloud, 0.5, 8);

  EXPECT_EQ(pcl_cloud.size(), cloud.size());
  for (size_t i = 0; i < pcl_cloud.size(); i++) {
    EXPECT_TRUE(pcl_cloud.at(i).cov.allFinite());
  }
}

// Normals estimated together with the covariances must follow the plane normal
TEST(VoxelCovarianceTest, PlaneNormals) {
  const PointCloud cloud = make_plane_cloud(2000);

  for (int method = 0; method < 2; method++) {
    PointCloud estimated = cloud;
    if (method == 0) {
      estimate_voxel_raw_normals_covariances(estimated, 0.5);
    } else {
      estimate_voxel_filter_normals_covariances(estimated, 0.5, 8);
    }

    size_t num_checked = 0;
    for (size_t i = 0; i < estimated.size(); i++) {
      const Eigen::Vector4d normal = estimated.normal(i);
      if (normal.norm() < 0.5) {
        continue;
      }
      EXPECT_NEAR(std::abs(normal.z()), 1.0, 1e-2);
      EXPECT_NEAR(normal.w(), 0.0, 1e-12);
      num_checked++;
    }
    EXPECT_GT(num_checked, 100);
  }
}

// Every point in the same voxel must share one normal
TEST(VoxelCovarianceTest, SameVoxelSameNormal) {
  const PointCloud cloud = make_plane_cloud(2000);
  const double leaf_size = 0.5;

  for (int method = 0; method < 2; method++) {
    PointCloud estimated = cloud;
    if (method == 0) {
      estimate_voxel_raw_normals_covariances(estimated, leaf_size);
    } else {
      estimate_voxel_filter_normals_covariances(estimated, leaf_size, 8);
    }

    std::map<std::array<int, 3>, Eigen::Vector4d> normal_of_voxel;
    size_t num_compared = 0;
    for (size_t i = 0; i < estimated.size(); i++) {
      const Eigen::Array3i coord = (estimated.point(i).head<3>() / leaf_size).array().floor().cast<int>();
      const std::array<int, 3> key = {coord[0], coord[1], coord[2]};
      if (estimated.normal(i).norm() < 0.5) {
        continue;  // No valid normal in this voxel
      }
      const auto found = normal_of_voxel.find(key);
      if (found == normal_of_voxel.end()) {
        normal_of_voxel.emplace(key, estimated.normal(i));
      } else {
        // The orientation is per point (towards the origin), so only the direction is compared
        EXPECT_NEAR(std::abs(found->second.dot(estimated.normal(i))), 1.0, 1e-9);
        num_compared++;
      }
    }
    EXPECT_GT(num_compared, 100);
  }
}

// Voxels with too few points have no valid normal
TEST(VoxelCovarianceTest, SparseVoxelNormal) {
  PointCloud cloud;
  cloud.resize(4);
  for (size_t i = 0; i < 4; i++) {
    cloud.point(i) << 100.0 + 0.1 * i, 100.0, 100.0, 1.0;
  }

  PointCloud estimated = cloud;
  estimate_voxel_raw_normals_covariances(estimated, 1.0);

  for (size_t i = 0; i < estimated.size(); i++) {
    EXPECT_LT(estimated.normal(i).norm(), 1e-12);
  }
}

// Multi-threaded and single-threaded normal+covariance estimation must give the same result
TEST(VoxelCovarianceTest, SerialEqualsOmpWithNormals) {
  const PointCloud cloud = make_plane_cloud(5000);

  for (int method = 0; method < 2; method++) {
    PointCloud serial = cloud;
    PointCloud omp = cloud;
    if (method == 0) {
      estimate_voxel_raw_normals_covariances(serial, 0.5);
      estimate_voxel_raw_normals_covariances_omp(omp, 0.5, 4);
    } else {
      estimate_voxel_filter_normals_covariances(serial, 0.5, 8);
      estimate_voxel_filter_normals_covariances_omp(omp, 0.5, 8, 4);
    }

    for (size_t i = 0; i < cloud.size(); i++) {
      EXPECT_LT((serial.normal(i) - omp.normal(i)).norm(), 1e-12);
      EXPECT_LT((serial.cov(i) - omp.cov(i)).norm(), 1e-12);
    }
  }
}

// The normal+covariance variants must work on PCL point clouds that have both fields
TEST(VoxelCovarianceTest, PclPointNormalCloud) {
  const PointCloud cloud = make_plane_cloud(1000);

  pcl::PointCloud<pcl::PointNormalCovariance> pcl_cloud;
  pcl_cloud.resize(cloud.size());
  for (size_t i = 0; i < cloud.size(); i++) {
    pcl_cloud.at(i).getVector4fMap() = cloud.point(i).cast<float>();
  }

  estimate_voxel_raw_normals_covariances(pcl_cloud, 0.5);
  estimate_voxel_filter_normals_covariances(pcl_cloud, 0.5, 8);

  for (size_t i = 0; i < pcl_cloud.size(); i++) {
    EXPECT_TRUE(pcl_cloud.at(i).cov.allFinite());
    EXPECT_NEAR(std::abs(pcl_cloud.at(i).getNormalVector4fMap().z()), 1.0, 1e-2);
  }
}
