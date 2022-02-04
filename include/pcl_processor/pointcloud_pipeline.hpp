/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Author: Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#pragma once

#include <unordered_map>
#include <memory>
#include <string>
#include <typeinfo>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "pcl_processor/pointcloud_processor.hpp"

#include <pluginlib/class_loader.hpp>

namespace pcl_processor
{
// TODO(shrijitsingh99): Look into a cosntexpr map
std::unordered_map<const std::type_info *, std::string> supported_point_types = {
  {&typeid(pcl::PointXYZ), "pcl::PointXYZ"}
};


template<typename PointT>
class PointCloudPipeline : public rclcpp::Node
{
public:
  using CloudT = pcl::PointCloud<PointT>;

  PointCloudPipeline()
  : Node("pointcloud_pipeline"), point_type_(supported_point_types[&typeid(PointT)]),
    processor_loader_("pcl_processor", "pcl_processor::PointCloudProcessor<" + point_type_ + ">")
  {
    // TODO(shrijitsingh99): Add pipeline name
    const auto input_topic = declare_parameter<std::string>("input_topic", "/points");
    const auto output_topic = declare_parameter<std::string>("output_topic", "~/filtered_points");

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, rclcpp::SensorDataQoS(),
      std::bind(&PointCloudPipeline::cloud_callback, this, std::placeholders::_1));
    processed_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic,
      rclcpp::SensorDataQoS());
  }

  void initialize()
  {
    auto node = shared_from_this();

    auto processor_plugins = declare_parameter<std::vector<std::string>>("processor_plugins");

    std::string processor_type;
    for (const auto & processor_plugin_name : processor_plugins) {
      try {
        processor_type = declare_parameter<std::string>(processor_plugin_name + ".plugin");
        processor_type += "<" + point_type_ + ">";

        processors_[processor_plugin_name] = processor_loader_.createSharedInstance(processor_type);
        RCLCPP_INFO(
          get_logger(), "Created processor %s of type %s",
          processor_plugin_name.c_str(), processor_type.c_str());

        processors_ordered_.push_back(processor_plugin_name);
        processors_[processor_plugin_name]->initialize(node, processor_plugin_name);
      } catch (const pluginlib::PluginlibException & ex) {
        RCLCPP_FATAL(
          get_logger(), "Failed to create processor %s of type %s. Exception: %s",
          processor_plugin_name.c_str(), processor_type.c_str(), ex.what());
      }
    }
  }

private:
  void cloud_callback(const sensor_msgs::msg::PointCloud2::UniquePtr msg)
  {
    // TODO(shrijitsingh99): Look into using circular buffer for pre-allocating pointclouds
    std::shared_ptr<CloudT> input_cloud_ = std::make_shared<CloudT>();
    pcl::fromROSMsg(*msg, *input_cloud_);

    for (const auto & processor_name : processors_ordered_) {
      auto & processor = processors_[processor_name];
      input_cloud_ = processor->process(input_cloud_);
    }

    sensor_msgs::msg::PointCloud2 filter_msg{};
    pcl::toROSMsg(*input_cloud_, filter_msg);
    filter_msg.header.frame_id = msg->header.frame_id;
    filter_msg.header.stamp = now();
    processed_cloud_pub_->publish(filter_msg);
  }

  const std::string point_type_;
  pluginlib::ClassLoader<PointCloudProcessor<PointT>> processor_loader_;
  std::unordered_map<std::string, typename PointCloudProcessor<PointT>::SharedPtr> processors_;
  std::vector<std::string> processors_ordered_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr processed_cloud_pub_;
};
}  // namespace pcl_processor
