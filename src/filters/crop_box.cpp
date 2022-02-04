/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Author: Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#include <algorithm>
#include <string>
#include <vector>
#include <utility>

#include "pcl_processor/filters/crop_box.hpp"

namespace pcl_processor
{
template<typename PointT>
typename CropBox<PointT>::PointCloudPtr CropBox<PointT>::process(
  CropBox<PointT>::PointCloudPtr cloud_in)
{
  processor_.setInputCloud(cloud_in);
  processor_.filter(*cloud_in);
  return cloud_in;
}

template<typename PointT>
void CropBox<PointT>::declare_parameters(rclcpp::Node::SharedPtr & node)
{
  param_callback_ = node->add_on_set_parameters_callback(
    [&](auto && arg)
    {
      return set_parameters(std::forward<decltype(arg)>(arg));
    });

  node->declare_parameter<std::vector<double>>(plugin_name_ + ".min", {-1.0, -1.0, -1.0});
  node->declare_parameter<std::vector<double>>(plugin_name_ + ".max", {1.0, 1.0, 1.0});
  node->declare_parameter<bool>(plugin_name_ + ".negative", false);
}

template<typename PointT>
rcl_interfaces::msg::SetParametersResult
CropBox<PointT>::set_parameters(const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  (void)parameters;
  for (const auto & parameter : parameters) {
    // TODO(shrijitsingh99): Add try-catch for catching any parameter exceptions and set
    // parameter result to false
    const std::string & parameter_name = parameter.get_name();
    if (parameter_name == plugin_name_ + ".min") {
      auto min = parameter.as_double_array();
      processor_.setMin(
        {static_cast<float>(min[0]), static_cast<float>(min[1]), static_cast<float>(min[2]), 1.0f});
    } else if (parameter_name == plugin_name_ + ".max") {
      auto max = parameter.as_double_array();
      processor_.setMax(
        {static_cast<float>(max[0]), static_cast<float>(max[1]), static_cast<float>(max[2]), 1.0f});
    } else if (parameter_name == plugin_name_ + ".negative") {
      processor_.setNegative(parameter.as_bool());
    }
  }
  return result;
}
}  // namespace pcl_processor

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  pcl_processor::CropBox<pcl::PointXYZ>,
  pcl_processor::PointCloudProcessor<pcl::PointXYZ>)
