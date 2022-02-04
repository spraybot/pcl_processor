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

#include <pcl/filters/experimental/functor_filter.h>
#include <pcl/point_types.h>

namespace pcl_processor
{
// TODO(shrijitsingh99): Add transform features present in pcl::CropBox also look into
// putting in a PR upstream into PCL for FunctorFilter based CropBox
template<typename PointT>
class CropBoxFilter : public pcl::experimental::FunctionFilter<PointT>
{
public:
  CropBoxFilter()
  : pcl::experimental::FunctionFilter<PointT>(std::bind(
        &CropBoxFilter::filter_function, this, std::placeholders::_1,
        std::placeholders::_2)) {}

  void setMin(const Eigen::Vector4f & min_pt)
  {
    min_pt_ = min_pt;
  }

  void setMax(const Eigen::Vector4f & max_pt)
  {
    max_pt_ = max_pt;
  }

private:
  Eigen::Vector4f min_pt_;
  Eigen::Vector4f max_pt_;

  bool filter_function(const pcl::PointCloud<PointT> & cloud, pcl::index_t idx)
  {
    const auto & pt = cloud[idx];
    return (pt.x > min_pt_[0] && pt.y > min_pt_[1] && pt.z > min_pt_[2]) &&
           (pt.x < max_pt_[0] && pt.y < max_pt_[1] && pt.z < max_pt_[2]);
  }
};

template<typename PointT>
class CropBox : public PointCloudProcessor<PointT>
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
  CropBoxFilter<PointT> processor_;
};
}  // namespace pcl_processor
