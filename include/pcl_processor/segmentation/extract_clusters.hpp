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
  void initialize() override
  {
    rclcpp::Node::SharedPtr node_ptr = this->node_.lock();

    cluster_marker_pub_ =
      node_ptr->create_publisher<visualization_msgs::msg::Marker>(
      std::string("~/") + this->plugin_name_ + "/clusters",
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
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;
  pcl::EuclideanClusterExtraction<PointT> processor_;
  typename pcl::search::KdTree<PointT>::Ptr kdtree_;
  std::vector<pcl::PointIndices> cluster_indices_;

  bool publish_markers_;
  std::string marker_frame_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cluster_marker_pub_;
};
}  // namespace pcl_processor
