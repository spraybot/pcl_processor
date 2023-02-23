/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Author: Shrijit Singh <shrijitsingh99@gmail.com>
 *
 * Implementation adapted from https://github.com/VincentCheungM/Run_based_segmentation
 *
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "pcl_processor/pointcloud_processor.hpp"

#include <rclcpp/rclcpp.hpp>

#include <pcl/filters/filter.h>
#include <pcl/common/centroid.h>


namespace pcl_processor
{
namespace impl
{

/*
 * Model parameter for ground plane fitting
 * The ground plane model is: ax + by + cz + d = 0
 * normal = [a,b,c], d = d
 * dist_threshold = plane_dist_threshold - d
 */
struct ModelParameters
{
  Eigen::MatrixXf normal;
  float d;
  float dist_threshold;
};

template<typename PointT>
class GroundPlaneFittingImpl
{
protected:
  using PointCloud = typename pcl::PointCloud<PointT>;
  using PointCloudPtr = typename PointCloud::Ptr;
  using PointCloudConstPtr = typename PointCloud::ConstPtr;

public:
  GroundPlaneFittingImpl()
  : num_iterations(1), num_lpr(1), initial_seeds_threshold(1.0), plane_dist_threshold(0.1),
    negative(false)
  {}

  PointCloudPtr fit(PointCloudPtr input_cloud);

  unsigned int num_iterations;
  unsigned int num_lpr;
  double initial_seeds_threshold;
  double plane_dist_threshold;
  bool negative;

private:
  void estimate_plane(const PointCloud & seed_points);

  void extract_initial_seeds(PointCloud & sorted_points);

  ModelParameters model_;
};
}  // namespace impl

template<typename PointT>
class GroundPlaneFitting : public PointCloudProcessor<PointT>
{
protected:
  using PointCloud = typename PointCloudProcessor<PointT>::PointCloud;
  using PointCloudPtr = typename PointCloudProcessor<PointT>::PointCloudPtr;
  using PointCloudConstPtr = typename PointCloudProcessor<PointT>::PointCloudConstPtr;

public:
  void initialize() override
  {
    rclcpp::Node::SharedPtr node_ptr = this->node_.lock();
    declare_parameters(node_ptr);
  }

  PointCloudPtr process(PointCloudPtr cloud_in) override;

private:
  void declare_parameters(rclcpp::Node::SharedPtr & node);

  rcl_interfaces::msg::SetParametersResult set_parameters(
    const std::vector<rclcpp::Parameter> & parameters);

protected:
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;
  impl::GroundPlaneFittingImpl<PointT> processor_;
};
}  // namespace pcl_processor
