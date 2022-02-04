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

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/point_types.h>

namespace pcl_processor
{
template<typename PointT>
class RadiusOutlierRemoval : public PointCloudProcessor<PointT>
{
protected:
  using PointCloud = typename PointCloudProcessor<PointT>::PointCloud;
  using PointCloudPtr = typename PointCloudProcessor<PointT>::PointCloudPtr;
  using PointCloudConstPtr = typename PointCloudProcessor<PointT>::PointCloudConstPtr;

public:
  void initialize(rclcpp::Node::WeakPtr node, const std::string & plugin_name) override
  {
    node_ = node;
    plugin_name_ = plugin_name;
    auto node_ptr = node.lock();

    declare_parameters(node_ptr);
  }

  PointCloudPtr process(PointCloudPtr cloud_in) override;

private:
  void declare_parameters(rclcpp::Node::SharedPtr & node);

  rcl_interfaces::msg::SetParametersResult set_parameters(
    const std::vector<rclcpp::Parameter> & parameters);

protected:
  std::string plugin_name_;
  rclcpp::Node::WeakPtr node_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;
  pcl::RadiusOutlierRemoval<PointT> processor_;
};
}  // namespace pcl_processor
