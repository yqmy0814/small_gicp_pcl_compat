# small_gicp

English | [中文](README_zh.md)

**small_gicp** is a header-only C++ library providing efficient and parallelized algorithms for fine point cloud registration (ICP, Point-to-Plane ICP, GICP, VGICP, etc.). It is a refined and optimized version of its predecessor, [fast_gicp](https://github.com/SMRT-AIST/fast_gicp), re-written from scratch with the following features.

- **Highly Optimized** : The core registration algorithm implementation has been further optimized from fast_gicp, achieving up to **2x speed gain**.
- **Fully parallerized** : small_gicp offers parallel implementations of several preprocessing algorithms, making the entire registration process parallelized (e.g., Downsampling, KdTree construction, Normal/Covariance estimation). It supports [OpenMP](https://www.openmp.org/) and [Intel TBB](https://github.com/oneapi-src/oneTBB) as parallelism backends.
- **Minimum dependencies** : The library requires only [Eigen](https://eigen.tuxfamily.org/) along with the bundled [nanoflann](https://github.com/jlblancoc/nanoflann) and [Sophus](https://github.com/strasdat/Sophus). Optionally, it supports a [PCL](https://pointclouds.org/) interface that accepts `pcl::PointCloud` directly and is not tied to a specific PCL version
- **Customizable** : small_gicp allows the integration of any custom point cloud class into the registration algorithm via traits. Its template-based implementation enables customization of the registration process with original correspondence estimators and registration factors.
- **Python bindings** : By being isolated from PCL, small_gicp's Python bindings are more portable and can be used seamlessly with other libraries such as Open3D.

Note that GPU-based implementations are NOT included in this package.

If you find this package useful for your project, please consider leaving a comment [here](https://github.com/koide3/small_gicp/issues/3). It would help the author receive recognition in his organization and keep working on this project. Please also cite [the corresponding software paper](https://joss.theoj.org/papers/10.21105/joss.06948) if you use this software in an academic work.


[![status](https://joss.theoj.org/papers/059b017c823ed9fd211fc91373cdc2cc/status.svg)](https://joss.theoj.org/papers/059b017c823ed9fd211fc91373cdc2cc) [![Build(Linux)](https://github.com/koide3/small_gicp/actions/workflows/build-linux.yml/badge.svg)](https://github.com/koide3/small_gicp/actions/workflows/build-linux.yml) [![macos](https://github.com/koide3/small_gicp/actions/workflows/build-macos.yml/badge.svg)](https://github.com/koide3/small_gicp/actions/workflows/build-macos.yml) [![Build(Windows)](https://github.com/koide3/small_gicp/actions/workflows/build-windows.yml/badge.svg)](https://github.com/koide3/small_gicp/actions/workflows/build-windows.yml) [![Test](https://github.com/koide3/small_gicp/actions/workflows/test.yml/badge.svg)](https://github.com/koide3/small_gicp/actions/workflows/test.yml) [![codecov](https://codecov.io/gh/koide3/small_gicp/graph/badge.svg?token=PCVIUP2Z33)](https://codecov.io/gh/koide3/small_gicp)

## Changes from the upstream project

This repository is a fork of [small_gicp](https://github.com/koide3/small_gicp). On top of the upstream features, it contains the following changes.

### PCL version independence

- The PCL interface no longer depends on the PCL version. The `pcl::Registration`-derived wrapper (`RegistrationPCL`, `pcl/pcl_registration.hpp`) has been removed; `pcl/pcl.hpp` is the single entry header and provides the PCL point types, the traits specialization, the point cloud conversion helpers (`toEigenVector` / `fromEigenVector`), `PointCloudProxy`, and an `align()` overload that takes `pcl::PointCloud`.
- The PCL 1.11 requirement (a `static_assert` on `pcl::shared_ptr`) has been removed, so both the `boost::shared_ptr` based (PCL 1.10) and the `std::shared_ptr` based (PCL 1.11 and later) point cloud pointers can be used.
- Link-type keywords (`optimized`, `debug`, `general`) are stripped from `PCL_LIBRARIES` before they are written into an imported target, so `-DBUILD_WITH_PCL=ON` configures with PCL versions whose `PCLConfig.cmake` does not provide a `PCL::PCL` target.
- The PCL example, the PCL odometry benchmark and the PCL-related tests were ported to the new interface, and a test for the `align()` overload was added.
- Verified with PCL 1.10 (Ubuntu 20.04) and PCL 1.12.1 (Ubuntu 22.04): all tests pass on both.

### Voxel-based covariance initialization

- `util/voxel_covariance.hpp` provides two additional covariance estimation methods that assign one covariance to all points in a voxel: `estimate_voxel_raw_covariances[_omp]` estimates it from the points inside the voxel, and `estimate_voxel_filter_covariances[_omp]` from the nearest voxel centroids. The `_normals_covariances` variants write normals as well. Both methods are much cheaper than the per-point estimation and are parallelized over voxels.
- `RegistrationSetting` gains `cov_init` (`POINT_KNN` by default, `VOXEL_RAW`, `VOXEL_FILTER`), `cov_voxel_size`, `cov_num_neighbors`, and `num_neighbors`; `preprocess_points(points, setting)` selects the estimator accordingly. The previous `preprocess_points(points, downsampling_resolution, num_neighbors, num_threads)` is kept as a wrapper, so its behavior does not change.
- `src/example/05_covariance_init.cpp` shows how to select the initialization and compares the preprocessing cost and the accuracy of the methods.


**The recommended setting is `VOXEL_FILTER`.** It keeps 0% weak constraints at every downsampling resolution and works with both VGICP and GICP, so it is a safe default for general use.

| `cov_init` | How it works | Parameters | Pros | Cons |
|---|---|---|---|---|
| `POINT_KNN` (default) | One covariance per point from its nearest neighbors | `num_neighbors` (10), `num_threads` | The upstream behavior and no extra parameter beyond `num_neighbors`; accurate on downsampled clouds (27.9 mm at 0.10 m, 19.6 / 14.5 mm at 0.25 / 0.50 m) | The per-point nearest neighbor search dominates the preprocessing (35.6 ms of covariance estimation and 92.8 ms in total on the raw cloud, about five times `VOXEL_FILTER`); the scale follows the point density, and on the dense raw cloud the 10th neighbor is only 62 mm away, which makes its error the largest of the three (124.8 mm) |
| `VOXEL_RAW` | One covariance per voxel from the points inside the voxel, no nearest neighbor search | `cov_voxel_size` (0.5 m), `num_threads` | The cheapest of the three (0.07-3.10 ms) and a single parameter; accurate enough on dense clouds (3.1% weak constraints on the raw cloud) | It requires at least 5 points per voxel, so the weak constraint ratio grows quickly as the cloud gets sparser (83.3% at 0.25 m, 99.8% at 0.50 m) and turns directly into a bias with GICP (42.96 mm and 452.28 mm); the voxel size therefore has to grow with the point spacing (keeping the weak ratio below 10% needs 1.5 m at 0.25 m and more than 2 m at 0.50 m), so one voxel size cannot serve clouds of different densities; a voxel may mix several surfaces, which distorts the covariance at corners |
| `VOXEL_FILTER` | One covariance per voxel from the nearest voxel centroids, assigned to all points in the voxel | `cov_voxel_size` (0.5 m), `cov_num_neighbors` (8), `num_threads` | It is cheap (preprocessing 1.3-4.1 ms) and the better covariance means fewer iterations, so the total time is the lowest or close to the lowest; it does not fall back to the weak constraint at any point density (0% weak constraints at every downsampling resolution) | One more parameter (`cov_num_neighbors`, tuned together with `cov_voxel_size`) and a slightly higher covariance estimation cost than `VOXEL_RAW`; its accuracy depends on the voxel size (12.4 mm at 1.0 m vs. 60.8 mm at 2.0 m on the 0.25 m downsampled cloud), so `cov_voxel_size` still has to follow the downsampling resolution (41.3 mm with a 0.5 m voxel on the raw cloud), but it adapts better to different point densities than `VOXEL_RAW`, which falls back to the weak constraint once its voxels are too small |

#### Experimental results

Both tables are produced by `src/example/05_covariance_init.cpp` with no initial offset and 4 threads (set `downsampling_resolutions = {0.0, 0.1, 0.25, 0.5}` and `cov_voxel_sizes = {0.5, 1.0}` in `BenchmarkConfig` to reproduce them); the ground truth is a translation of 504.3 mm and a rotation of 0.716 deg. `cov[ms]` is the covariance estimation time of the target and source clouds, `reg[ms]` includes the target-side structure construction (voxelmap or kdtree), `total[ms]` is `cov[ms] + reg[ms]`, and `weak[%]` is the ratio of target points that received the weak constraint (`Identity`) instead of a valid covariance. The numbers are from a single run; the multi-threaded downsampling and reduction make them vary slightly between runs.

#### VGICP

| Downsampling | `cov_init` | Setting | cov[ms] | reg[ms] | total[ms] | trans[mm] | rot[deg] | weak[%] |
|---|---|---|---|---|---|---|---|---|
| raw (69088 / 69792) | `VOXEL_RAW` | 0.5 m | 3.10 | 18.58 | 21.68 | 24.054 | 0.243 | 3.1 |
| raw (69088 / 69792) | `VOXEL_RAW` | 1.0 m | **2.85** | 15.46 | 18.31 | **19.182** | **0.185** | 1.1 |
| raw (69088 / 69792) | `VOXEL_FILTER` | 0.5 m, k=8 | 4.05 | **13.08** | **17.13** | 41.338 | 0.537 | **0.0** |
| raw (69088 / 69792) | `VOXEL_FILTER` | 1.0 m, k=8 | 3.02 | 14.21 | 17.24 | 58.599 | 0.521 | **0.0** |
| raw (69088 / 69792) | `POINT_KNN` | k=10 | 35.59 | 57.24 | 92.83 | 124.827 | 0.292 | **0.0** |
| 0.10 m (15828 / 16004) | `VOXEL_RAW` | 0.5 m | 0.58 | 3.26 | 3.85 | 29.188 | 0.357 | 20.2 |
| 0.10 m (15828 / 16004) | `VOXEL_RAW` | 1.0 m | **0.56** | 3.13 | **3.69** | 14.508 | **0.168** | 6.0 |
| 0.10 m (15828 / 16004) | `VOXEL_FILTER` | 0.5 m, k=8 | 1.63 | **2.78** | 4.41 | **11.798** | 0.240 | **0.0** |
| 0.10 m (15828 / 16004) | `VOXEL_FILTER` | 1.0 m, k=8 | 0.80 | 3.84 | 4.65 | 13.825 | 0.294 | **0.0** |
| 0.10 m (15828 / 16004) | `POINT_KNN` | k=10 | 7.90 | 3.69 | 11.59 | 27.945 | 0.661 | **0.0** |
| 0.25 m (6206 / 6230) | `VOXEL_RAW` | 0.5 m | **0.20** | 4.07 | 4.27 | 24.336 | 0.559 | 83.3 |
| 0.25 m (6206 / 6230) | `VOXEL_RAW` | 1.0 m | 0.26 | **1.18** | **1.44** | **11.767** | **0.104** | 21.6 |
| 0.25 m (6206 / 6230) | `VOXEL_FILTER` | 0.5 m, k=8 | 1.42 | 1.29 | 2.71 | 15.572 | 0.195 | **0.0** |
| 0.25 m (6206 / 6230) | `VOXEL_FILTER` | 1.0 m, k=8 | 0.60 | 1.40 | 2.00 | 12.436 | 0.290 | **0.0** |
| 0.25 m (6206 / 6230) | `POINT_KNN` | k=10 | 3.04 | 1.33 | 4.37 | 19.602 | 0.378 | **0.0** |
| 0.50 m (2747 / 2721) | `VOXEL_RAW` | 0.5 m | **0.07** | 1.91 | 1.98 | 23.254 | 0.490 | 99.8 |
| 0.50 m (2747 / 2721) | `VOXEL_RAW` | 1.0 m | 0.11 | 0.85 | **0.96** | 292.557 | 1.012 | 72.8 |
| 0.50 m (2747 / 2721) | `VOXEL_FILTER` | 0.5 m, k=8 | 1.30 | 0.66 | 1.96 | 22.093 | **0.218** | **0.0** |
| 0.50 m (2747 / 2721) | `VOXEL_FILTER` | 1.0 m, k=8 | 0.52 | 0.81 | 1.33 | 25.795 | 0.369 | **0.0** |
| 0.50 m (2747 / 2721) | `POINT_KNN` | k=10 | 1.31 | **0.64** | 1.95 | **20.529** | 0.241 | **0.0** |

#### GICP

| Downsampling | `cov_init` | Setting | cov[ms] | reg[ms] | total[ms] | trans[mm] | rot[deg] | weak[%] |
|---|---|---|---|---|---|---|---|---|
| raw (69088 / 69792) | `VOXEL_RAW` | 0.5 m | 2.89 | 91.81 | 94.71 | 290.555 | 1.121 | 3.1 |
| raw (69088 / 69792) | `VOXEL_RAW` | 1.0 m | **2.83** | 60.09 | 62.92 | 283.008 | 0.949 | 1.1 |
| raw (69088 / 69792) | `VOXEL_FILTER` | 0.5 m, k=8 | 4.07 | **57.68** | **61.74** | **34.404** | **0.499** | **0.0** |
| raw (69088 / 69792) | `VOXEL_FILTER` | 1.0 m, k=8 | 3.19 | 79.63 | 82.82 | 54.374 | 0.526 | **0.0** |
| raw (69088 / 69792) | `POINT_KNN` | k=10 | 35.95 | 107.85 | 143.80 | 417.316 | 0.629 | **0.0** |
| 0.10 m (15828 / 16004) | `VOXEL_RAW` | 0.5 m | 0.57 | **5.20** | **5.77** | 3.308 | 0.153 | 20.2 |
| 0.10 m (15828 / 16004) | `VOXEL_RAW` | 1.0 m | **0.56** | 5.23 | 5.79 | 7.208 | 0.054 | 6.0 |
| 0.10 m (15828 / 16004) | `VOXEL_FILTER` | 0.5 m, k=8 | 1.63 | 5.26 | 6.89 | 4.530 | 0.398 | **0.0** |
| 0.10 m (15828 / 16004) | `VOXEL_FILTER` | 1.0 m, k=8 | 0.78 | 6.27 | 7.05 | 22.258 | 0.202 | **0.0** |
| 0.10 m (15828 / 16004) | `POINT_KNN` | k=10 | 7.94 | 6.35 | 14.29 | **0.001** | **0.000** | **0.0** |
| 0.25 m (6206 / 6230) | `VOXEL_RAW` | 0.5 m | **0.20** | 2.21 | 2.41 | 42.956 | 0.255 | 83.3 |
| 0.25 m (6206 / 6230) | `VOXEL_RAW` | 1.0 m | 0.26 | 1.83 | **2.09** | 11.957 | **0.140** | 21.6 |
| 0.25 m (6206 / 6230) | `VOXEL_FILTER` | 0.5 m, k=8 | 1.43 | 2.21 | 3.64 | 7.978 | 0.492 | **0.0** |
| 0.25 m (6206 / 6230) | `VOXEL_FILTER` | 1.0 m, k=8 | 0.61 | 2.21 | 2.82 | 22.463 | 0.312 | **0.0** |
| 0.25 m (6206 / 6230) | `POINT_KNN` | k=10 | 3.08 | **1.51** | 4.60 | **6.867** | 0.332 | **0.0** |
| 0.50 m (2747 / 2721) | `VOXEL_RAW` | 0.5 m | **0.07** | 1.14 | 1.21 | 452.276 | 0.892 | 99.8 |
| 0.50 m (2747 / 2721) | `VOXEL_RAW` | 1.0 m | 0.11 | 0.97 | **1.08** | 354.854 | 0.861 | 72.8 |
| 0.50 m (2747 / 2721) | `VOXEL_FILTER` | 0.5 m, k=8 | 1.27 | 1.47 | 2.74 | **9.822** | 0.700 | **0.0** |
| 0.50 m (2747 / 2721) | `VOXEL_FILTER` | 1.0 m, k=8 | 0.52 | 0.97 | 1.50 | 25.148 | 0.373 | **0.0** |
| 0.50 m (2747 / 2721) | `POINT_KNN` | k=10 | 1.31 | **0.96** | 2.27 | 10.087 | **0.278** | **0.0** |

## Requirements

This library uses C++17 features. The PCL interface no longer has a PCL version restriction: it works with any PCL version as long as the point cloud interfaces it relies on do not change significantly. It has been verified with PCL 1.10 and PCL 1.12.1.

## Dependencies

- [Mandatory] [Eigen](https://eigen.tuxfamily.org/), [nanoflann](https://github.com/jlblancoc/nanoflann) ([bundled](include/small_gicp/ann/kdtree.hpp)), [Sophus](https://github.com/strasdat/Sophus) ([bundled](include/small_gicp/util/lie.hpp))
- [Optional] [OpenMP](https://www.openmp.org/), [Intel TBB](https://www.intel.com/content/www/us/en/developer/tools/oneapi/onetbb.html), [PCL](https://pointclouds.org/)

## Installation

### C++

small_gicp is a header-only library. You can just download and drop it in your project directory to use it.

If you need only basic point cloud registration functions, you can build and install the helper library as follows.

```bash
sudo apt-get install libeigen3-dev libomp-dev

cd small_gicp
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j
sudo make install
```

### Python (Linux / Windows / MacOS)

#### Install from [PyPI](https://pypi.org/project/small-gicp/)

```bash
pip install small_gicp
```

#### Install from source

```bash
cd small_gicp
pip install .

# [Optional (linux)] Install stubs for autocomplete (If you know a better way, let me know...)
pip install pybind11-stubgen
cd ~/.local/lib/python3.10/site-packages
pybind11-stubgen -o . --ignore-invalid=all small_gicp
```

## Documentation

- Auto-generated docs: [C++](https://koide3.github.io/small_gicp/doc_cpp/index.html) [Python](https://koide3.github.io/small_gicp/doc_py/index.html)



## Usage (C++)

The following examples assume `using namespace small_gicp` is placed somewhere.

### Using helper library ([01_basic_registration.cpp](src/example/01_basic_registration.cpp))

The helper library (`registration_helper.hpp`) enables easily processing point clouds represented as `std::vector<Eigen::Vector(3|4)(f|d)>`.
<details><summary>Expand</summary>

`small_gicp::align` takes two point clouds (`std::vectors` of `Eigen::Vector(3|4)(f|d)`) and returns a registration result (estimated transformation and some information on the optimization result). This is the easiest way to use small_gicp but causes an overhead for duplicated preprocessing.

```cpp
#include <small_gicp/registration/registration_helper.hpp>

std::vector<Eigen::Vector3d> target_points = ...;   // Any of Eigen::Vector(3|4)(f|d) can be used
std::vector<Eigen::Vector3d> source_points = ...;   // 

RegistrationSetting setting;
setting.num_threads = 4;                    // Number of threads to be used
setting.downsampling_resolution = 0.25;     // Downsampling resolution
setting.max_correspondence_distance = 1.0;  // Maximum correspondence distance between points (e.g., triming threshold)

Eigen::Isometry3d init_T_target_source = Eigen::Isometry3d::Identity();
RegistrationResult result = align(target_points, source_points, init_T_target_source, setting);

Eigen::Isometry3d T = result.T_target_source;  // Estimated transformation
size_t num_inliers = result.num_inliers;       // Number of inlier source points
Eigen::Matrix<double, 6, 6> H = result.H;      // Final Hessian matrix (6x6)
```

There is also a way to perform preprocessing and registration separately. This enables saving time for preprocessing in case registration is performed several times for the same point cloud (e.g., typical odometry estimation based on scan-to-scan matching).

```cpp
#include <small_gicp/registration/registration_helper.hpp>

std::vector<Eigen::Vector3d> target_points = ...;   // Any of Eigen::Vector(3|4)(f|d) can be used
std::vector<Eigen::Vector3d> source_points = ...;   // 

int num_threads = 4;                    // Number of threads to be used
double downsampling_resolution = 0.25;  // Downsampling resolution
int num_neighbors = 10;                 // Number of neighbor points used for normal and covariance estimation

// std::pair<PointCloud::Ptr, KdTree<PointCloud>::Ptr>
auto [target, target_tree] = preprocess_points(target_points, downsampling_resolution, num_neighbors, num_threads);
auto [source, source_tree] = preprocess_points(source_points, downsampling_resolution, num_neighbors, num_threads);

RegistrationSetting setting;
setting.num_threads = num_threads;
setting.max_correspondence_distance = 1.0;  // Maximum correspondence distance between points (e.g., triming threshold)

Eigen::Isometry3d init_T_target_source = Eigen::Isometry3d::Identity();
RegistrationResult result = align(*target, *source, *target_tree, init_T_target_source, setting);

Eigen::Isometry3d T = result.T_target_source;  // Estimated transformation
size_t num_inliers = result.num_inliers;       // Number of inlier source points
Eigen::Matrix<double, 6, 6> H = result.H;      // Final Hessian matrix (6x6)
```

</details>

### Using PCL interface ([02_basic_registration_pcl.cpp](src/example/02_basic_registration_pcl.cpp))

The PCL interface is provided by a single header, `small_gicp/pcl/pcl.hpp`. It does not depend on a specific PCL version and works with any PCL version unless the point cloud interfaces change significantly. It provides the PCL point types, the traits specialization, the conversion helpers, and an `align()` overload that takes `pcl::PointCloud`. Note that this fork does not provide a `pcl::Registration`-derived wrapper; use the interfaces below instead.

<details><summary>Expand</summary>

`small_gicp::align` accepts `pcl::PointCloud` directly. The point clouds are downsampled and their normals and covariances are estimated internally according to the given setting.

```cpp
#include <small_gicp/pcl/pcl.hpp>

pcl::PointCloud<pcl::PointXYZ> target = ...;
pcl::PointCloud<pcl::PointXYZ> source = ...;

// Default setting (GICP, downsampling 0.25 m, 10 neighbors, 4 threads)
RegistrationResult result = align(target, source);
Eigen::Isometry3d T = result.T_target_source;  // Estimated transformation

// Explicit setting (e.g., VGICP with a voxel resolution of 1.0 m)
RegistrationSetting setting;
setting.type = RegistrationSetting::VGICP;
setting.voxel_resolution = 1.0;
setting.downsampling_resolution = 0.25;
setting.num_neighbors = 20;
setting.num_threads = 4;
setting.max_correspondence_distance = 1.0;

result = align(target, source, Eigen::Isometry3d::Identity(), setting);
```

It is also possible to feed `pcl::PointCloud` to `small_gicp::Registration`. All preprocessed data are exposed in this way, so you can easily re-use them to obtain the best efficiency.

```cpp
#include <small_gicp/pcl/pcl.hpp>

pcl::PointCloud<pcl::PointXYZ> raw_target = ...;
pcl::PointCloud<pcl::PointXYZ> raw_source = ...;

// Downsample points and convert them into pcl::PointCloud<pcl::PointCovariance>.
auto target = voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(raw_target, 0.25);
auto source = voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(raw_source, 0.25);

// Estimate covariances of points.
const int num_threads = 4;
const int num_neighbors = 20;
estimate_covariances_omp(*target, num_neighbors, num_threads);
estimate_covariances_omp(*source, num_neighbors, num_threads);

// Create KdTree for target and source.
auto target_tree = std::make_shared<KdTree<pcl::PointCloud<pcl::PointCovariance>>>(target, KdTreeBuilderOMP(num_threads));
auto source_tree = std::make_shared<KdTree<pcl::PointCloud<pcl::PointCovariance>>>(source, KdTreeBuilderOMP(num_threads));

Registration<GICPFactor, ParallelReductionOMP> registration;
registration.reduction.num_threads = num_threads;
registration.rejector.max_dist_sq = 1.0;

// Align point clouds. Note that the input point clouds are pcl::PointCloud<pcl::PointCovariance>.
auto result = registration.align(*target, *source, *target_tree, Eigen::Isometry3d::Identity());
```

The conversion helpers are available when the point clouds have to be passed to the algorithms that accept only `std::vector<Eigen::Vector(3|4)(f|d)>`.

```cpp
// pcl::PointCloud -> std::vector<Eigen::Vector4d>, and back
std::vector<Eigen::Vector4d> points = toEigenVector(raw_target);
fromEigenVector(points, raw_target);
```

</details>

### Using `Registration` template ([03_registration_template.cpp](src/example/03_registration_template.cpp))

If you want to fine-control and customize the registration process, use `small_gicp::Registration` template that allows modifying the inner algorithms and parameters.
<details><summary>Expand</summary>

```cpp
#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>
#include <small_gicp/registration/reduction_omp.hpp>
#include <small_gicp/registration/registration.hpp>

std::vector<Eigen::Vector3d> target_points = ...;   // Any of Eigen::Vector(3|4)(f|d) can be used
std::vector<Eigen::Vector3d> source_points = ...;   // 

int num_threads = 4;
double downsampling_resolution = 0.25;
int num_neighbors = 10;
double max_correspondence_distance = 1.0;

// Convert to small_gicp::PointCloud
auto target = std::make_shared<PointCloud>(target_points);
auto source = std::make_shared<PointCloud>(source_points);

// Downsampling
target = voxelgrid_sampling_omp(*target, downsampling_resolution, num_threads);
source = voxelgrid_sampling_omp(*source, downsampling_resolution, num_threads);

// Create KdTree
auto target_tree = std::make_shared<KdTree<PointCloud>>(target, KdTreeBuilderOMP(num_threads));
auto source_tree = std::make_shared<KdTree<PointCloud>>(source, KdTreeBuilderOMP(num_threads));

// Estimate point covariances
estimate_covariances_omp(*target, *target_tree, num_neighbors, num_threads);
estimate_covariances_omp(*source, *source_tree, num_neighbors, num_threads);

// GICP + OMP-based parallel reduction
Registration<GICPFactor, ParallelReductionOMP> registration;
registration.reduction.num_threads = num_threads;
registration.rejector.max_dist_sq = max_correspondence_distance * max_correspondence_distance;

// Align point clouds
Eigen::Isometry3d init_T_target_source = Eigen::Isometry3d::Identity();
auto result = registration.align(*target, *source, *target_tree, init_T_target_source);

Eigen::Isometry3d T = result.T_target_source;  // Estimated transformation
size_t num_inliers = result.num_inliers;       // Number of inlier source points
Eigen::Matrix<double, 6, 6> H = result.H;      // Final Hessian matrix (6x6)
```

See [03_registration_template.cpp](src/example/03_registration_template.cpp)  for more detailed customization examples.

</details>

### Cookbook

- [Standard scan-to-scan GICP matching odometry](src/benchmark/odometry_benchmark_small_gicp_omp.cpp)
- [Extremely scalable scan-to-scan matching odometry with data flow graph](src/benchmark/odometry_benchmark_small_gicp_tbb_flow.cpp)
- [Scan-to-model matching odometry with incremental voxelmap (GICP + iVox)](src/benchmark/odometry_benchmark_small_gicp_model_tbb.cpp)
- [Scan-to-model matching odometry with incremental Gaussian voxelmap (VGICP)](src/benchmark/odometry_benchmark_small_vgicp_model_tbb.cpp)

## Usage (Python) [basic_registration.py](src/example/basic_registration.py)

<details><summary>Expand</summary>

Example A : Perform registration with numpy arrays

```python
# Align two point clouds using various ICP-like algorithms.
# 
# Parameters
# ----------
# target_points : NDArray[np.float64]
#     Nx3 or Nx4 matrix representing the target point cloud.
# source_points : NDArray[np.float64]
#     Nx3 or Nx4 matrix representing the source point cloud.
# init_T_target_source : np.ndarray[np.float64]
#     4x4 matrix representing the initial transformation from target to source.
# registration_type : str = 'GICP'
#     Type of registration algorithm to use ('ICP', 'PLANE_ICP', 'GICP', 'VGICP').
# voxel_resolution : float = 1.0
#     Resolution of voxels used for correspondence search (used only in VGICP).
# downsampling_resolution : float = 0.25
#     Resolution for downsampling the point clouds.
# max_correspondence_distance : float = 1.0
#     Maximum distance for matching points between point clouds.
# num_threads : int = 1
#     Number of threads to use for parallel processing.
# 
# Returns
# -------
# RegistrationResult
#     Object containing the final transformation matrix and convergence status.
result = small_gicp.align(target_raw_numpy, source_raw_numpy, downsampling_resolution=0.25)

result.T_target_source  # Estimated transformation (4x4 numpy array)
result.converged        # If true, the optimization converged successfully
result.iterations       # Number of iterations the optimization took
result.num_inliers      # Number of inlier points
result.H                # Final Hessian matrix (6x6 matrix)
result.b                # Final information vector (6D vector)
result.e                # Final error (float)
```

Example B : Perform preprocessing and registration separately

```python
# Preprocess point cloud (downsampling, kdtree construction, and normal/covariance estimation)
#
# Parameters
# ----------
# points : NDArray[np.float64]
#     Nx3 or Nx4 matrix representing the point cloud.
# downsampling_resolution : float = 0.1
#     Resolution for downsampling the point clouds.
# num_neighbors : int = 20
#     Number of neighbor points to usefor point normal/covariance estimation.
# num_threads : int = 1
#     Number of threads to use for parallel processing.
# 
# Returns
# -------
# PointCloud
#     Downsampled point cloud with estimated normals and covariances.
# KdTree
#     KdTree for the downsampled point cloud
target, target_tree = small_gicp.preprocess_points(target_raw_numpy, downsampling_resolution=0.25)
source, source_tree = small_gicp.preprocess_points(source_raw_numpy, downsampling_resolution=0.25)

# `target` and `source` are small_gicp.PointCloud with the following methods
target.size()           # Number of points
target.points()         # Nx4 numpy array   [x, y, z, 1] x N
target.normals()        # Nx4 numpy array   [nx, ny, nz, 0] x N
target.covs()           # Array of 4x4 covariance matrices

#  Align two point clouds using specified ICP-like algorithms, utilizing point cloud and KD-tree inputs.
#
#  Parameters
#  ----------
#  target : PointCloud::ConstPtr
#      Pointer to the target point cloud.
#  source : PointCloud::ConstPtr
#      Pointer to the source point cloud.
#  target_tree : KdTree<PointCloud>::ConstPtr, optional
#      Pointer to the KD-tree of the target for nearest neighbor search. If nullptr, a new tree is built.
#  init_T_target_source : NDArray[np.float64]
#      4x4 matrix representing the initial transformation from target to source.
#  registration_type : str = 'GICP'
#      Type of registration algorithm to use ('ICP', 'PLANE_ICP', 'GICP').
#  max_correspondence_distance : float = 1.0
#      Maximum distance for corresponding point pairs.
#  num_threads : int = 1
#      Number of threads to use for computation.
# 
#  Returns
#  -------
#  RegistrationResult
#      Object containing the final transformation matrix and convergence status.
result = small_gicp.align(target, source, target_tree)
```

Example C : Perform each of preprocessing steps one-by-one

```python
# Convert numpy arrays (Nx3 or Nx4) to small_gicp.PointCloud
target_raw = small_gicp.PointCloud(target_raw_numpy)
source_raw = small_gicp.PointCloud(source_raw_numpy)

# Downsampling
target = small_gicp.voxelgrid_sampling(target_raw, 0.25)
source = small_gicp.voxelgrid_sampling(source_raw, 0.25)

# KdTree construction
target_tree = small_gicp.KdTree(target)
source_tree = small_gicp.KdTree(source)

# Estimate covariances
small_gicp.estimate_covariances(target, target_tree)
small_gicp.estimate_covariances(source, source_tree)

# Align point clouds
result = small_gicp.align(target, source, target_tree)
```

Example D: Example with Open3D

```python
target_o3d = open3d.io.read_point_cloud('small_gicp/data/target.ply').paint_uniform_color([0, 1, 0])
source_o3d = open3d.io.read_point_cloud('small_gicp/data/source.ply').paint_uniform_color([0, 0, 1])

target, target_tree = small_gicp.preprocess_points(numpy.asarray(target_o3d.points), downsampling_resolution=0.25)
source, source_tree = small_gicp.preprocess_points(numpy.asarray(source_o3d.points), downsampling_resolution=0.25)
result = small_gicp.align(target, source, target_tree)

source_o3d.transform(result.T_target_source)
open3d.visualization.draw_geometries([target_o3d, source_o3d])
```

</details>


### Cookbook

- [Scan-to-scan and scan-to-model GICP matching odometry on KITTI](src/example/kitti_odometry.py)

## Running examples

### C++

```bash
cd small_gicp
mkdir build && cd build
cmake .. -DBUILD_EXAMPLES=ON && make -j

cd ..
./build/01_basic_registration
./build/02_basic_registration_pcl
./build/03_registration_template
./build/04_colored_icp
./build/05_covariance_init
```


### Python

```bash
cd small_gicp
pip install .

python3 src/example/basic_registration.py
```

## [Benchmark](BENCHMARK.md)

Processing speed comparison between small_gicp and Open3D ([youtube](https://youtu.be/LNESzGXPr4c?feature=shared)).

### Downsampling

- Single-threaded `small_gicp::voxelgrid_sampling` is about **1.3x faster** than `pcl::VoxelGrid`.
- Multi-threaded `small_gicp::voxelgrid_sampling_tbb` (6 threads) is about **3.2x faster** than `pcl::VoxelGrid`.
- `small_gicp::voxelgrid_sampling` provides accurate downsampling results that are nearly identical to those of `pcl::VoxelGrid`, while `pcl::ApproximateVoxelGrid` can produce spurious points (up to 2x more points).
- `small_gicp::voxelgrid_sampling` can handle larger point clouds with finer voxel resolutions compared to `pcl::VoxelGrid`. For a point cloud with a width of 1000m, the minimum voxel resolution can be **0.5 mm**.

![downsampling_comp](docs/assets/downsampling_comp.png)

### KdTree construction

- Multi-threaded implementation (TBB and OMP) can be up to **6x faster** than the single-threaded version. The single-thread version performs almost equivalently to nanoflann.
- The new KdTree implementation demonstrates good scalability due to its well-balanced task assignment.
- This benchmark compares only the construction time (query time is not included). Nearest neighbor queries are included and evaluated in the following odometry estimation evaluation.

![kdtree_time](docs/assets/kdtree_time.png)

### Odometry estimation

- Single-thread `small_gicp::GICP` is about **2.4x and 1.9x faster** than `pcl::GICP` and `fast_gicp::GICP`, respectively.
- `small_gicp::(GICP|VGICP)` demonstrates better multi-threaded scalability compared to `fast_gicp::(GICP|VGICP)`.
- `small_gicp::GICP` parallelized with [TBB flow graph](src/benchmark/odometry_benchmark_small_gicp_tbb_flow.cpp) shows excellent scalability in many-threads scenarios (**~128 threads**), though with some latency degradation.
- Outputs of `small_gicp::GICP` are almost identical to those of `fast_gicp::GICP`.

![odometry_time](docs/assets/odometry_time.png)

## License
This package is released under the MIT license.

If you find this package useful for your project, please consider leaving a comment [here](https://github.com/koide3/small_gicp/issues/3). It would help the author receive recognition in his organization and keep working on this project.

Please also cite [the following article](https://joss.theoj.org/papers/10.21105/joss.06948) if you use this software in an academic work.

```
@article{small_gicp,
author = {Kenji Koide},
title = {{small\_gicp: Efficient and parallel algorithms for point cloud registration}},
journal = {Journal of Open Source Software},
month = aug,
number = {100},
pages = {6948},
volume = {9},
year = {2024},
doi = {10.21105/joss.06948}
}
```

## Contact

[Kenji Koide](https://staff.aist.go.jp/k.koide/), National Institute of Advanced Industrial Science and Technology (AIST)
