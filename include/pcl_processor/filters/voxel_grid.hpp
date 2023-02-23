/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Author: Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#pragma once

#include <string>
#include <vector>

#include "pcl_processor/pointcloud_processor.hpp"

#include <rclcpp/rclcpp.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>

namespace pcl_processor
{
template<typename PointT>
class VoxelGrid : public PointCloudProcessor<PointT>
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
  pcl::VoxelGrid<PointT> processor_;
};
}  // namespace pcl_processor
