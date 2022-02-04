/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Author: Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#include <memory>

#include "pcl_processor/pointcloud_pipeline.hpp"

#include <rclcpp/rclcpp.hpp>

#include <pcl/point_types.h>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<pcl_processor::PointCloudPipeline<pcl::PointXYZ>>();
  node->initialize();

  rclcpp::spin(node);

  return 0;
}
