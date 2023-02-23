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
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>

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

  void setup(rclcpp::Node::WeakPtr node, const std::string & plugin_name, std::shared_ptr<tf2_ros::Buffer> & tf_buffer) {
    node_ = node;
    plugin_name_ = plugin_name;
    tf_buffer_ = tf_buffer;
    rclcpp::Node::SharedPtr node_ptr = node.lock();

    const auto publish_flag = node_ptr->declare_parameter<bool>(plugin_name_ + ".publish", false);

    if (publish_flag) {
      auto publish_topic = node_ptr->declare_parameter<std::string>(plugin_name_ + ".publish_topic", "");

      if (publish_topic.empty()) {
        publish_topic = std::string(node_ptr->get_fully_qualified_name()) + "/" + plugin_name_ + "/cloud";
      }

      processed_cloud_pub_ = node_ptr->create_publisher<sensor_msgs::msg::PointCloud2>(
        publish_topic, rclcpp::SensorDataQoS());
    }
  }

  virtual void initialize() = 0;
  virtual PointCloudPtr process(PointCloudPtr cloud_in) = 0;

protected:
  rclcpp::Node::WeakPtr node_;
  std::string plugin_name_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr processed_cloud_pub_;
};
}  // namespace pcl_processor
