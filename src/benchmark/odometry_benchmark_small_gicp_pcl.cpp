#ifdef BUILD_WITH_PCL

#include <small_gicp/benchmark/benchmark_odom.hpp>
#include <small_gicp/pcl/pcl.hpp>

namespace small_gicp {

class SmallGICPPCLOdometryEstimation : public OnlineOdometryEstimation {
public:
  explicit SmallGICPPCLOdometryEstimation(const OdometryEstimationParams& params) : OnlineOdometryEstimation(params), T(Eigen::Isometry3d::Identity()) {}

  Eigen::Isometry3d estimate(const PointCloud::Ptr& points) override {
    // Feed PCL point clouds to the native registration through the traits specialization
    auto points_pcl = std::make_shared<pcl::PointCloud<pcl::PointCovariance>>();
    points_pcl->resize(points->size());
    for (size_t i = 0; i < points->size(); i++) {
      points_pcl->at(i).getVector4fMap() = points->point(i).template cast<float>();
    }

    Stopwatch sw;
    sw.start();

    // Preprocess input points (kdtree construction & covariance estimation)
    // Note that input points here is already downsampled
    auto tree = std::make_shared<KdTree<pcl::PointCloud<pcl::PointCovariance>>>(points_pcl, KdTreeBuilderOMP(params.num_threads));
    estimate_covariances_omp(*points_pcl, *tree, params.num_neighbors, params.num_threads);

    if (target_points == nullptr) {
      // This is the very first frame
      target_points = points_pcl;
      target_tree = tree;
      return T;
    }

    // Registration using GICP + OMP-based parallel reduction
    Registration<GICPFactor, ParallelReductionOMP> registration;
    registration.rejector.max_dist_sq = params.max_correspondence_distance * params.max_correspondence_distance;
    registration.reduction.num_threads = params.num_threads;

    const auto result = registration.align(*target_points, *points_pcl, *target_tree, Eigen::Isometry3d::Identity());

    sw.stop();
    reg_times.push(sw.msec());

    T = T * result.T_target_source;
    target_points = points_pcl;
    target_tree = tree;

    return T;
  }

  void report() override {  //
    std::cout << "registration_time_stats=" << reg_times.str() << " [msec/scan]  total_throughput=" << total_times.str() << " [msec/scan]" << std::endl;
  }

private:
  Summarizer reg_times;

  std::shared_ptr<pcl::PointCloud<pcl::PointCovariance>> target_points;             // Last point cloud to be registration target
  std::shared_ptr<KdTree<pcl::PointCloud<pcl::PointCovariance>>> target_tree;       // KdTree of the last target points

  Eigen::Isometry3d T;
};

static auto small_gicp_pcl_registry =
  register_odometry("small_gicp_pcl", [](const OdometryEstimationParams& params) { return std::make_shared<SmallGICPPCLOdometryEstimation>(params); });

}  // namespace small_gicp

#endif