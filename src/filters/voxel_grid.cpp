/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Author: Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#include <algorithm>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "pcl_processor/filters/voxel_grid.hpp"

namespace pcl_processor
{
template<typename PointT>
typename VoxelGrid<PointT>::PointCloudPtr VoxelGrid<PointT>::process(
  VoxelGrid<PointT>::PointCloudPtr cloud_in)
{
  processor_.setInputCloud(cloud_in);
  processor_.filter(*cloud_in);
  return cloud_in;
}

template<typename PointT>
void VoxelGrid<PointT>::declare_parameters(rclcpp::Node::SharedPtr & node)
{
  param_callback_ = node->add_on_set_parameters_callback(
    [&](auto && arg)
    {
      return set_parameters(std::forward<decltype(arg)>(arg));
    });

  node->declare_parameter<std::vector<double>>(plugin_name_ + ".leaf_size", {0.1, 0.1, 0.1});
  node->declare_parameter<bool>(plugin_name_ + ".downsample_all_data", true);
  node->declare_parameter<int64_t>(plugin_name_ + ".min_points_per_voxel", 0);
  node->declare_parameter<bool>(plugin_name_ + ".save_leaf_layout", false);
}

template<typename PointT>
rcl_interfaces::msg::SetParametersResult
VoxelGrid<PointT>::set_parameters(const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & parameter : parameters) {
    // TODO(shrijitsingh99): Add try-catch for catching any parameter exceptions and set
    // parameter result to false
    const std::string & parameter_name = parameter.get_name();
    if (parameter_name == plugin_name_ + ".leaf_size") {
      auto leaf_size = parameter.as_double_array();
      processor_.setLeafSize(leaf_size[0], leaf_size[1], leaf_size[2]);
    } else if (parameter_name == plugin_name_ + ".downsample_all_data") {
      processor_.setDownsampleAllData(parameter.as_bool());
    } else if (parameter_name == plugin_name_ + ".min_points_per_voxel") {
      processor_.setMinimumPointsNumberPerVoxel(
        static_cast<unsigned>(std::max(0l, parameter.as_int())));
    } else if (parameter_name == plugin_name_ + ".save_leaf_layout") {
      processor_.setSaveLeafLayout(parameter.as_bool());
    }
  }
  return result;
}
}  // namespace pcl_processor

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  pcl_processor::VoxelGrid<pcl::PointXYZ>,
  pcl_processor::PointCloudProcessor<pcl::PointXYZ>)
