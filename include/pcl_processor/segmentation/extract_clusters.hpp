/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Author: Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "pcl_processor/pointcloud_processor.hpp"

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>

namespace pcl_processor
{
template<typename PointT>
class EuclideanClusterExtraction : public PointCloudProcessor<PointT>
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

    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(node_ptr->get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cluster_marker_pub_ =
      node_ptr->create_publisher<visualization_msgs::msg::Marker>(
      "~/" + plugin_name + "/clusters",
      rclcpp::SystemDefaultsQoS());

    kdtree_ = std::make_shared<pcl::search::KdTree<PointT>>();
    processor_.setSearchMethod(kdtree_);

    declare_parameters(node_ptr);
  }

  PointCloudPtr process(PointCloudPtr cloud_in) override;

private:
  void declare_parameters(rclcpp::Node::SharedPtr & node);

  rcl_interfaces::msg::SetParametersResult set_parameters(
    const std::vector<rclcpp::Parameter> & parameters);

  void publish_clusters(
    PointCloudConstPtr cloud_in,
    const std::vector<pcl::PointIndices> & cluster_indices) const;

protected:
  std::string plugin_name_;
  rclcpp::Node::WeakPtr node_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  pcl::EuclideanClusterExtraction<PointT> processor_;
  typename pcl::search::KdTree<PointT>::Ptr kdtree_;
  std::vector<pcl::PointIndices> cluster_indices_;

  bool publish_markers_;
  std::string marker_frame_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cluster_marker_pub_;
};
}  // namespace pcl_processor
