# small_gicp

[English](README.md) | 中文

**small_gicp** 是一个 header-only 的 C++ 库，提供高效、并行化的精细点云配准算法（ICP、点到面 ICP、GICP、VGICP 等）。它是前身 [fast_gicp](https://github.com/SMRT-AIST/fast_gicp) 的重写版本，具有以下特点：

- **高度优化**：核心配准算法在 fast_gicp 基础上进一步优化，加速比最高可达 **2 倍**。
- **完全并行化**：small_gicp 为多项预处理算法提供并行实现，使整个配准流程（降采样、KdTree 构建、法向/协方差估计等）都能并行执行，支持 [OpenMP](https://www.openmp.org/) 与 [Intel TBB](https://github.com/oneapi-src/oneTBB) 两种并行后端。
- **依赖最少**：只需 [Eigen](https://eigen.tuxfamily.org/)，以及随仓库附带的 [nanoflann](https://github.com/jlblancoc/nanoflann) 与 [Sophus](https://github.com/strasdat/Sophus)。可选支持 [PCL](https://pointclouds.org/) 接口，可直接接收 `pcl::PointCloud`，不绑定特定 PCL 版本。
- **可定制**：通过 traits 可以把任意自定义点云类接入配准算法；基于模板的实现允许替换对应关系估计器与配准因子，从而定制配准流程。
- **Python 绑定**：由于与 PCL 解耦，Python 绑定更易移植，可与 Open3D 等库配合使用。

本库不包含基于 GPU 的实现。

如果这个库对你的项目有帮助，欢迎在[这里](https://github.com/koide3/small_gicp/issues/3)留言；在学术工作中使用本软件时，也请引用[对应的软件论文](https://joss.theoj.org/papers/10.21105/joss.06948)。


[![status](https://joss.theoj.org/papers/059b017c823ed9fd211fc91373cdc2cc/status.svg)](https://joss.theoj.org/papers/059b017c823ed9fd211fc91373cdc2cc) [![Build(Linux)](https://github.com/koide3/small_gicp/actions/workflows/build-linux.yml/badge.svg)](https://github.com/koide3/small_gicp/actions/workflows/build-linux.yml) [![macos](https://github.com/koide3/small_gicp/actions/workflows/build-macos.yml/badge.svg)](https://github.com/koide3/small_gicp/actions/workflows/build-macos.yml) [![Build(Windows)](https://github.com/koide3/small_gicp/actions/workflows/build-windows.yml/badge.svg)](https://github.com/koide3/small_gicp/actions/workflows/build-windows.yml) [![Test](https://github.com/koide3/small_gicp/actions/workflows/test.yml/badge.svg)](https://github.com/koide3/small_gicp/actions/workflows/test.yml) [![codecov](https://codecov.io/gh/koide3/small_gicp/graph/badge.svg?token=PCVIUP2Z33)](https://codecov.io/gh/koide3/small_gicp)

## 相对原项目的改动

本仓库是 [small_gicp](https://github.com/koide3/small_gicp) 的 fork。在上游功能之外，相对原项目做了以下改动。

### PCL 版本无关

- PCL 接口不再依赖 PCL 版本。派生自 `pcl::Registration` 的封装类（`RegistrationPCL`、`pcl/pcl_registration.hpp`）已删除；`pcl/pcl.hpp` 是唯一入口头文件，提供 PCL 点类型、traits 特化、点云转换辅助函数（`toEigenVector` / `fromEigenVector`）、`PointCloudProxy`，以及接受 `pcl::PointCloud` 的 `align()` 重载。
- 去掉了对 PCL 1.11 的强制要求（针对 `pcl::shared_ptr` 的 `static_assert`），因此基于 `boost::shared_ptr`（PCL 1.10）与基于 `std::shared_ptr`（PCL 1.11 及以后）的点云指针都可以使用。
- 写入导入目标前，先从 `PCL_LIBRARIES` 中剔除链接类型关键字（`optimized`、`debug`、`general`），使得 `PCLConfig.cmake` 不提供 `PCL::PCL` 目标的 PCL 版本也能正常配置 `-DBUILD_WITH_PCL=ON`。
- PCL 示例、PCL 里程计基准与相关测试都迁移到了新接口，并新增了一个针对 `align()` 重载的测试。
- 已在 PCL 1.10（Ubuntu 20.04）与 PCL 1.12.1（Ubuntu 22.04）上验证：两个环境全部测试通过。

### 体素协方差初始化

- `util/voxel_covariance.hpp` 提供两种额外的协方差估计方式，都是"每个体素算一个协方差，赋给该体素内所有点"：`estimate_voxel_raw_covariances[_omp]` 用体素内原始点估计，`estimate_voxel_filter_covariances[_omp]` 用最近的体素质心估计；带 `_normals_covariances` 的变体会同时写入法向。两种方式都比逐点估计便宜得多，并且按体素并行。
- `RegistrationSetting` 新增 `cov_init`（默认 `POINT_KNN`，另有 `VOXEL_RAW`、`VOXEL_FILTER`）、`cov_voxel_size`、`cov_num_neighbors` 与 `num_neighbors`；`preprocess_points(points, setting)` 据此选择估计方式。原有的 `preprocess_points(points, downsampling_resolution, num_neighbors, num_threads)` 保留为包装函数，行为不变。
- `src/example/05_covariance_init.cpp` 演示如何选择这些初始化方式，并对比各方式的预处理耗时与精度。


**推荐使用 `VOXEL_FILTER`。** 它在所有降采样档位都保持 0% 弱约束，且在 VGICP 与 GICP 下都可用，适合作为通用场景的默认设置。

| `cov_init` | 计算方式 | 参数 | 优势 | 劣势 |
|---|---|---|---|---|
| `POINT_KNN`（默认）| 每个点用最近的近邻点估计一个协方差 | `num_neighbors`（10）、`num_threads` | 上游默认行为，除 `num_neighbors` 外无需额外参数；降采样点云上精度好（0.10 m 档 27.9 mm，0.25 / 0.50 m 档 19.6 / 14.5 mm）| 逐点近邻搜索是预处理主要开销（原始点云协方差估计 35.6 ms、总耗时 92.8 ms，约为 `VOXEL_FILTER` 的 5 倍）；尺度随点云密度变化，原始稠密点云上第 10 近邻只有 62 mm，误差反而最大（124.8 mm）|
| `VOXEL_RAW` | 每个体素用内部原始点估计一个协方差，不做近邻搜索 | `cov_voxel_size`（0.5 m）、`num_threads` | 三种方式里最便宜（0.07-3.10 ms），只需一个参数；稠密点云上精度可用（原始点云弱约束 3.1%）| 要求体素内至少 5 个点，点云变稀疏后弱约束比例迅速上升（0.25 m 档 83.3%、0.50 m 档 99.8%），且在 GICP 下直接变成偏置（42.96 mm 与 452.28 mm）；为使弱约束低于 10%，0.25 m 档需约 1.5 m 体素、0.50 m 档需 2 m 以上，因此同一个体素尺寸无法同时适配不同密度的点云；体素内可能混合多个面，角点处协方差会失真 |
| `VOXEL_FILTER` | 每个体素用最近的体素质心估计一个协方差，再赋给该体素内所有点 | `cov_voxel_size`（0.5 m）、`cov_num_neighbors`（8）、`num_threads` | 预处理便宜（1.3-4.1 ms），协方差质量更好带来更少迭代，总耗时最低或接近最低；各档位均不退化（0% 弱约束） | 多一个参数（`cov_num_neighbors`，需与 `cov_voxel_size` 一起调），协方差估计耗时略高于 `VOXEL_RAW`；精度仍受体素尺寸影响（0.25 m 降采样云上 1.0 m 为 12.4 mm、2.0 m 为 60.8 mm），`cov_voxel_size` 仍需跟着降采样分辨率调整（原始点云上用 0.5 m 体素为 41.3 mm），但对不同密度点云的适应性强于 `VOXEL_RAW` —— 后者体素过小时会退化为弱约束|

#### 实验结果

两张表均由 `src/example/05_covariance_init.cpp` 生成，无初始偏移、4 线程（在 `BenchmarkConfig` 中设置 `downsampling_resolutions = {0.0, 0.1, 0.25, 0.5}` 与 `cov_voxel_sizes = {0.5, 1.0}` 即可复现），真值为平移 504.3 mm、旋转 0.716°。`cov[ms]` 为目标点云与源点云协方差估计的合计耗时，`reg[ms]` 含目标侧结构（体素图或 kdtree）构建，`total[ms]` 为 `cov[ms] + reg[ms]`，`weak[%]` 为目标点云中退化为弱约束（`Identity`）而非有效协方差的点数比例。表中数据为单次运行结果，多线程降采样与并行归约会使其在多次运行间略有波动。

#### VGICP

| 降采样 | `cov_init` | 设置 | cov[ms] | reg[ms] | total[ms] | 平移[mm] | 旋转[deg] | weak[%] |
|---|---|---|---|---|---|---|---|---|
| 不滤波（69088 / 69792） | `VOXEL_RAW` | 0.5 m | 3.10 | 18.58 | 21.68 | 24.054 | 0.243 | 3.1 |
| 不滤波（69088 / 69792） | `VOXEL_RAW` | 1.0 m | **2.85** | 15.46 | 18.31 | **19.182** | **0.185** | 1.1 |
| 不滤波（69088 / 69792） | `VOXEL_FILTER` | 0.5 m, k=8 | 4.05 | **13.08** | **17.13** | 41.338 | 0.537 | **0.0** |
| 不滤波（69088 / 69792） | `VOXEL_FILTER` | 1.0 m, k=8 | 3.02 | 14.21 | 17.24 | 58.599 | 0.521 | **0.0** |
| 不滤波（69088 / 69792） | `POINT_KNN` | k=10 | 35.59 | 57.24 | 92.83 | 124.827 | 0.292 | **0.0** |
| 0.10 m（15828 / 16004） | `VOXEL_RAW` | 0.5 m | 0.58 | 3.26 | 3.85 | 29.188 | 0.357 | 20.2 |
| 0.10 m（15828 / 16004） | `VOXEL_RAW` | 1.0 m | **0.56** | 3.13 | **3.69** | 14.508 | **0.168** | 6.0 |
| 0.10 m（15828 / 16004） | `VOXEL_FILTER` | 0.5 m, k=8 | 1.63 | **2.78** | 4.41 | **11.798** | 0.240 | **0.0** |
| 0.10 m（15828 / 16004） | `VOXEL_FILTER` | 1.0 m, k=8 | 0.80 | 3.84 | 4.65 | 13.825 | 0.294 | **0.0** |
| 0.10 m（15828 / 16004） | `POINT_KNN` | k=10 | 7.90 | 3.69 | 11.59 | 27.945 | 0.661 | **0.0** |
| 0.25 m（6206 / 6230） | `VOXEL_RAW` | 0.5 m | **0.20** | 4.07 | 4.27 | 24.336 | 0.559 | 83.3 |
| 0.25 m（6206 / 6230） | `VOXEL_RAW` | 1.0 m | 0.26 | **1.18** | **1.44** | **11.767** | **0.104** | 21.6 |
| 0.25 m（6206 / 6230） | `VOXEL_FILTER` | 0.5 m, k=8 | 1.42 | 1.29 | 2.71 | 15.572 | 0.195 | **0.0** |
| 0.25 m（6206 / 6230） | `VOXEL_FILTER` | 1.0 m, k=8 | 0.60 | 1.40 | 2.00 | 12.436 | 0.290 | **0.0** |
| 0.25 m（6206 / 6230） | `POINT_KNN` | k=10 | 3.04 | 1.33 | 4.37 | 19.602 | 0.378 | **0.0** |
| 0.50 m（2747 / 2721） | `VOXEL_RAW` | 0.5 m | **0.07** | 1.91 | 1.98 | 23.254 | 0.490 | 99.8 |
| 0.50 m（2747 / 2721） | `VOXEL_RAW` | 1.0 m | 0.11 | 0.85 | **0.96** | 292.557 | 1.012 | 72.8 |
| 0.50 m（2747 / 2721） | `VOXEL_FILTER` | 0.5 m, k=8 | 1.30 | 0.66 | 1.96 | 22.093 | **0.218** | **0.0** |
| 0.50 m（2747 / 2721） | `VOXEL_FILTER` | 1.0 m, k=8 | 0.52 | 0.81 | 1.33 | 25.795 | 0.369 | **0.0** |
| 0.50 m（2747 / 2721） | `POINT_KNN` | k=10 | 1.31 | **0.64** | 1.95 | **20.529** | 0.241 | **0.0** |

#### GICP

| 降采样 | `cov_init` | 设置 | cov[ms] | reg[ms] | total[ms] | 平移[mm] | 旋转[deg] | weak[%] |
|---|---|---|---|---|---|---|---|---|
| 不滤波（69088 / 69792） | `VOXEL_RAW` | 0.5 m | 2.89 | 91.81 | 94.71 | 290.555 | 1.121 | 3.1 |
| 不滤波（69088 / 69792） | `VOXEL_RAW` | 1.0 m | **2.83** | 60.09 | 62.92 | 283.008 | 0.949 | 1.1 |
| 不滤波（69088 / 69792） | `VOXEL_FILTER` | 0.5 m, k=8 | 4.07 | **57.68** | **61.74** | **34.404** | **0.499** | **0.0** |
| 不滤波（69088 / 69792） | `VOXEL_FILTER` | 1.0 m, k=8 | 3.19 | 79.63 | 82.82 | 54.374 | 0.526 | **0.0** |
| 不滤波（69088 / 69792） | `POINT_KNN` | k=10 | 35.95 | 107.85 | 143.80 | 417.316 | 0.629 | **0.0** |
| 0.10 m（15828 / 16004） | `VOXEL_RAW` | 0.5 m | 0.57 | **5.20** | **5.77** | 3.308 | 0.153 | 20.2 |
| 0.10 m（15828 / 16004） | `VOXEL_RAW` | 1.0 m | **0.56** | 5.23 | 5.79 | 7.208 | 0.054 | 6.0 |
| 0.10 m（15828 / 16004） | `VOXEL_FILTER` | 0.5 m, k=8 | 1.63 | 5.26 | 6.89 | 4.530 | 0.398 | **0.0** |
| 0.10 m（15828 / 16004） | `VOXEL_FILTER` | 1.0 m, k=8 | 0.78 | 6.27 | 7.05 | 22.258 | 0.202 | **0.0** |
| 0.10 m（15828 / 16004） | `POINT_KNN` | k=10 | 7.94 | 6.35 | 14.29 | **0.001** | **0.000** | **0.0** |
| 0.25 m（6206 / 6230） | `VOXEL_RAW` | 0.5 m | **0.20** | 2.21 | 2.41 | 42.956 | 0.255 | 83.3 |
| 0.25 m（6206 / 6230） | `VOXEL_RAW` | 1.0 m | 0.26 | 1.83 | **2.09** | 11.957 | **0.140** | 21.6 |
| 0.25 m（6206 / 6230） | `VOXEL_FILTER` | 0.5 m, k=8 | 1.43 | 2.21 | 3.64 | 7.978 | 0.492 | **0.0** |
| 0.25 m（6206 / 6230） | `VOXEL_FILTER` | 1.0 m, k=8 | 0.61 | 2.21 | 2.82 | 22.463 | 0.312 | **0.0** |
| 0.25 m（6206 / 6230） | `POINT_KNN` | k=10 | 3.08 | **1.51** | 4.60 | **6.867** | 0.332 | **0.0** |
| 0.50 m（2747 / 2721） | `VOXEL_RAW` | 0.5 m | **0.07** | 1.14 | 1.21 | 452.276 | 0.892 | 99.8 |
| 0.50 m（2747 / 2721） | `VOXEL_RAW` | 1.0 m | 0.11 | 0.97 | **1.08** | 354.854 | 0.861 | 72.8 |
| 0.50 m（2747 / 2721） | `VOXEL_FILTER` | 0.5 m, k=8 | 1.27 | 1.47 | 2.74 | **9.822** | 0.700 | **0.0** |
| 0.50 m（2747 / 2721） | `VOXEL_FILTER` | 1.0 m, k=8 | 0.52 | 0.97 | 1.50 | 25.148 | 0.373 | **0.0** |
| 0.50 m（2747 / 2721） | `POINT_KNN` | k=10 | 1.31 | **0.96** | 2.27 | 10.087 | **0.278** | **0.0** |

## 环境要求

本库使用 C++17 特性。PCL 接口已不再有 PCL 版本限制：只要它依赖的点云相关接口没有出现显著变化，各版本均可使用。已在 PCL 1.10 与 PCL 1.12.1 上验证。

## 依赖

- [必需] [Eigen](https://eigen.tuxfamily.org/)、[nanoflann](https://github.com/jlblancoc/nanoflann)（[已附带](include/small_gicp/ann/kdtree.hpp)）、[Sophus](https://github.com/strasdat/Sophus)（[已附带](include/small_gicp/util/lie.hpp)）
- [可选] [OpenMP](https://www.openmp.org/)、[Intel TBB](https://www.intel.com/content/www/us/en/developer/tools/oneapi/onetbb.html)、[PCL](https://pointclouds.org/)

## 安装

### C++

small_gicp 是 header-only 库，下载后直接放进项目目录即可使用。

如果只需要基础的点云配准功能，也可以构建并安装辅助库：

```bash
sudo apt-get install libeigen3-dev libomp-dev

cd small_gicp
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j
sudo make install
```

### Python（Linux / Windows / MacOS）

#### 通过 [PyPI](https://pypi.org/project/small-gicp/) 安装

```bash
pip install small_gicp
```

#### 从源码安装

```bash
cd small_gicp
pip install .

# [Optional (linux)] Install stubs for autocomplete (If you know a better way, let me know...)
pip install pybind11-stubgen
cd ~/.local/lib/python3.10/site-packages
pybind11-stubgen -o . --ignore-invalid=all small_gicp
```

## 文档

- 自动生成的文档：[C++](https://koide3.github.io/small_gicp/doc_cpp/index.html) [Python](https://koide3.github.io/small_gicp/doc_py/index.html)



## C++ 用法

以下示例假定某处已声明 `using namespace small_gicp`。

### 使用辅助库（[01_basic_registration.cpp](src/example/01_basic_registration.cpp)）

辅助库（`registration_helper.hpp`）可以直接处理用 `std::vector<Eigen::Vector(3|4)(f|d)>` 表示的点云。
<details><summary>展开</summary>

`small_gicp::align` 接收两朵点云（`std::vector<Eigen::Vector(3|4)(f|d)>`）并返回配准结果（估计的变换以及优化结果的一些信息）。这是使用 small_gicp 最简单的方式，代价是预处理会被重复执行。

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

也可以把预处理与配准分开执行。当同一朵点云需要参与多次配准（例如典型的 scan-to-scan 里程计）时，这样可以省下重复的预处理时间。

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

### 使用 PCL 接口（[02_basic_registration_pcl.cpp](src/example/02_basic_registration_pcl.cpp)）

PCL 接口由单个头文件 `small_gicp/pcl/pcl.hpp` 提供，不绑定特定的 PCL 版本，只要点云相关接口没有出现显著变化，各版本均可使用。它提供 PCL 点类型、traits 特化、转换辅助函数，以及接受 `pcl::PointCloud` 的 `align()` 重载。本 fork 不提供派生自 `pcl::Registration` 的封装类，请使用下面这些接口。

<details><summary>展开</summary>

`small_gicp::align` 可以直接接收 `pcl::PointCloud`。点云会在内部按给定配置完成降采样以及法向、协方差估计。

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

也可以把 `pcl::PointCloud` 直接送入 `small_gicp::Registration`。这种方式下所有预处理数据都是显式暴露的，便于复用，效率最好。

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

当点云需要送入只接受 `std::vector<Eigen::Vector(3|4)(f|d)>` 的算法时，可以使用转换辅助函数。

```cpp
// pcl::PointCloud -> std::vector<Eigen::Vector4d>, and back
std::vector<Eigen::Vector4d> points = toEigenVector(raw_target);
fromEigenVector(points, raw_target);
```

</details>

### 使用 `Registration` 模板（[03_registration_template.cpp](src/example/03_registration_template.cpp)）

如果需要精细控制并定制配准流程，可以使用 `small_gicp::Registration` 模板，它能替换内部算法与参数。
<details><summary>展开</summary>

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

更详细的定制示例见 [03_registration_template.cpp](src/example/03_registration_template.cpp)。

</details>

### 示例集

- [标准的 scan-to-scan GICP 匹配里程计](src/benchmark/odometry_benchmark_small_gicp_omp.cpp)
- [基于数据流图的超大规模 scan-to-scan 匹配里程计](src/benchmark/odometry_benchmark_small_gicp_tbb_flow.cpp)
- [基于增量体素图的 scan-to-model 匹配里程计（GICP + iVox）](src/benchmark/odometry_benchmark_small_gicp_model_tbb.cpp)
- [基于增量高斯体素图的 scan-to-model 匹配里程计（VGICP）](src/benchmark/odometry_benchmark_small_vgicp_model_tbb.cpp)

## Python 用法（[basic_registration.py](src/example/basic_registration.py)）

<details><summary>展开</summary>

示例 A：用 numpy 数组执行配准

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

示例 B：预处理与配准分开执行

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

示例 C：逐步执行每个预处理步骤

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

示例 D：与 Open3D 配合使用

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


### 示例集

- [KITTI 上的 scan-to-scan 与 scan-to-model GICP 匹配里程计](src/example/kitti_odometry.py)

## 运行示例

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

## [基准测试](BENCHMARK.md)

small_gicp 与 Open3D 的处理速度对比（[youtube](https://youtu.be/LNESzGXPr4c?feature=shared)）。

### 降采样

- 单线程 `small_gicp::voxelgrid_sampling` 比 `pcl::VoxelGrid` **快约 1.3 倍**。
- 多线程 `small_gicp::voxelgrid_sampling_tbb`（6 线程）比 `pcl::VoxelGrid` **快约 3.2 倍**。
- `small_gicp::voxelgrid_sampling` 的降采样结果与 `pcl::VoxelGrid` 几乎一致，而 `pcl::ApproximateVoxelGrid` 可能产生虚假点（最多多出一倍的点）。
- 相比 `pcl::VoxelGrid`，`small_gicp::voxelgrid_sampling` 能用更细的体素分辨率处理更大的点云。对于宽度 1000 m 的点云，最小体素分辨率可达 **0.5 mm**。

![downsampling_comp](docs/assets/downsampling_comp.png)

### KdTree 构建

- 多线程实现（TBB 与 OMP）相比单线程版本最多**快 6 倍**；单线程版本与 nanoflann 性能几乎一致。
- 新的 KdTree 实现由于任务划分均衡，具备良好的扩展性。
- 该基准只比较构建时间（不含查询时间）。最近邻查询的开销在后面的里程计评测中被计入并评估。

![kdtree_time](docs/assets/kdtree_time.png)

### 里程计估计

- 单线程 `small_gicp::GICP` 分别比 `pcl::GICP` 与 `fast_gicp::GICP` **快约 2.4 倍和 1.9 倍**。
- 相比 `fast_gicp::(GICP|VGICP)`，`small_gicp::(GICP|VGICP)` 的多线程扩展性更好。
- 用 [TBB flow graph](src/benchmark/odometry_benchmark_small_gicp_tbb_flow.cpp) 并行化的 `small_gicp::GICP` 在多线程场景（**约 128 线程**）下表现出优异的扩展性，但延迟会有所上升。
- `small_gicp::GICP` 的输出与 `fast_gicp::GICP` 几乎一致。

![odometry_time](docs/assets/odometry_time.png)

## 许可证

本包以 MIT 许可证发布。

如果这个库对你的项目有帮助，欢迎在[这里](https://github.com/koide3/small_gicp/issues/3)留言；这将帮助作者在其所在机构获得认可，并持续维护该项目。

在学术工作中使用本软件时，也请引用[以下文章](https://joss.theoj.org/papers/10.21105/joss.06948)。

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

## 联系方式

[Kenji Koide](https://staff.aist.go.jp/k.koide/)，日本产业技术综合研究所（AIST）
