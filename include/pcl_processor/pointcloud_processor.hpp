/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Author: Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/macros.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pcl_processor
{
template<typename PointT>
class PointCloudProcessor
{
protected:
  PointCloudProcessor() = default;

  using PointCloud = pcl::PointCloud<PointT>;
  using PointCloudPtr = typename PointCloud::Ptr;
  using PointCloudConstPtr = typename PointCloud::ConstPtr;

public:
  RCLCPP_SMART_PTR_DEFINITIONS(PointCloudProcessor<PointT>)

  virtual ~PointCloudProcessor() = default;

  virtual void initialize(rclcpp::Node::WeakPtr node, const std::string & plugin_name) = 0;
  virtual PointCloudPtr process(PointCloudPtr cloud_in) = 0;
};
}  // namespace pcl_processor
