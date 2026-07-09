// NDT usability check: builds the map from the test world, synthesizes a
// scan taken from a known sensor pose, then verifies PCL's NDT recovers
// that pose from a perturbed initial guess. This guards the property the
// tool exists for: maps whose voxel statistics are good enough for NDT
// localization.
#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include <Eigen/Geometry>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>

#include "sdf2map/options.hpp"
#include "sdf2map/world_sampler.hpp"

namespace
{

sdf2map::CloudXYZ::Ptr VoxelFilter(
  const sdf2map::CloudXYZ::Ptr & in, float leaf)
{
  auto out = std::make_shared<sdf2map::CloudXYZ>();
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(in);
  voxel.setLeafSize(leaf, leaf, leaf);
  voxel.filter(*out);
  return out;
}

}  // namespace

TEST(NdtLocalization, RecoversSensorPoseOnGeneratedMap)
{
  sdf2map::Options opts;
  opts.input = std::string(TEST_WORLDS_DIR) + "/test_world.sdf";
  opts.density = 400.0;
  sdf2map::SampleStats stats;
  const auto raw = sdf2map::SampleWorld(opts, stats);

  // Map as it would be exported for NDT
  const auto map = VoxelFilter(raw, 0.1f);
  ASSERT_GT(map->size(), 10000u);

  // Ground-truth sensor pose in the world
  const Eigen::Vector3f gt_pos(1.0f, 0.5f, 0.0f);
  const float gt_yaw = 0.3f;
  Eigen::Affine3f gt = Eigen::Translation3f(gt_pos) *
    Eigen::AngleAxisf(gt_yaw, Eigen::Vector3f::UnitZ());

  // Synthetic scan: map points within sensor range, expressed in the
  // sensor frame (a lidar-like observation without occlusion modelling)
  auto scan = std::make_shared<sdf2map::CloudXYZ>();
  const Eigen::Affine3f gt_inv = gt.inverse();
  for (const auto & p : *map) {
    const float range = std::hypot(p.x - gt_pos.x(), p.y - gt_pos.y());
    if (range > 12.0f || p.z > 2.5f) {
      continue;
    }
    scan->push_back(pcl::transformPoint(p, gt_inv));
  }
  ASSERT_GT(scan->size(), 5000u);

  // Parameters validated by sweep: resolution 2.0 converges to ~2 cm in
  // <10 iterations on this map; epsilon must be small enough not to cut
  // the line search off early (0.01 stalls near the initial guess).
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
  ndt.setResolution(2.0f);
  ndt.setStepSize(0.1);
  ndt.setTransformationEpsilon(0.001);
  ndt.setMaximumIterations(100);
  ndt.setInputTarget(map);
  ndt.setInputSource(scan);

  // Initial guess perturbed from the truth: 0.4 m and ~9 deg off
  Eigen::Affine3f guess = Eigen::Translation3f(
    gt_pos + Eigen::Vector3f(0.4f, -0.3f, 0.0f)) *
    Eigen::AngleAxisf(gt_yaw + 0.15f, Eigen::Vector3f::UnitZ());

  sdf2map::CloudXYZ aligned;
  ndt.align(aligned, guess.matrix());

  ASSERT_TRUE(ndt.hasConverged());
  const Eigen::Affine3f result(ndt.getFinalTransformation());

  const float pos_err = (result.translation() - gt.translation()).norm();
  const Eigen::AngleAxisf rot_err(result.rotation().transpose() *
    gt.rotation());
  EXPECT_LT(pos_err, 0.05f) << "NDT position error too large";
  EXPECT_LT(std::abs(rot_err.angle()), 0.02f) << "NDT rotation error";
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
