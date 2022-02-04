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

#include "pcl_processor/filters/radius_outlier_removal.hpp"

namespace pcl_processor
{
template<typename PointT>
typename RadiusOutlierRemoval<PointT>::PointCloudPtr RadiusOutlierRemoval<PointT>::process(
  RadiusOutlierRemoval<PointT>::PointCloudPtr cloud_in)
{
  processor_.setInputCloud(cloud_in);
  processor_.filter(*cloud_in);
  return cloud_in;
}

template<typename PointT>
void RadiusOutlierRemoval<PointT>::declare_parameters(rclcpp::Node::SharedPtr & node)
{
  param_callback_ = node->add_on_set_parameters_callback(
    [&](auto && arg)
    {
      return set_parameters(std::forward<decltype(arg)>(arg));
    });

  node->declare_parameter<double>(plugin_name_ + ".radius_search", 0.0);
  node->declare_parameter<int64_t>(plugin_name_ + ".min_points_per_voxel", 1);
}

template<typename PointT>
rcl_interfaces::msg::SetParametersResult
RadiusOutlierRemoval<PointT>::set_parameters(const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & parameter : parameters) {
    // TODO(shrijitsingh99): Add try-catch for catching any parameter exceptions and set
    // parameter result to false
    const std::string & parameter_name = parameter.get_name();
    if (parameter_name == plugin_name_ + ".radius_search") {
      processor_.setRadiusSearch(parameter.as_double());
    } else if (parameter_name == plugin_name_ + ".min_neighbors_in_radius") {
      processor_.setMinNeighborsInRadius(
        std::min(0u, static_cast<unsigned>(parameter.as_int())));
    }
  }
  return result;
}
}  // namespace pcl_processor

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  pcl_processor::RadiusOutlierRemoval<pcl::PointXYZ>,
  pcl_processor::PointCloudProcessor<pcl::PointXYZ>)
