/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Author: Shrijit Singh <shrijitsingh99@gmail.com>
 *
 * Implementation adapted from https://github.com/VincentCheungM/Run_based_segmentation
 *
 */

#include <algorithm>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "pcl_processor/segmentation/ground_plane_fitting.hpp"

namespace pcl_processor
{
namespace impl
{
template<typename PointT>
typename GroundPlaneFittingImpl<PointT>::PointCloudPtr GroundPlaneFittingImpl<PointT>::fit(
  PointCloudPtr input_cloud)
{
  pcl::PointCloud<PointT> input_cloud_original, g_not_ground_pc;
  g_not_ground_pc.header = input_cloud->header;
  g_not_ground_pc.reserve(input_cloud->size());
  copyPointCloud(*input_cloud, input_cloud_original);

  // TODO(shrijitsingh99): Add segmenting of pointcloud and fitting multiple ground planes

  // TODO(shrijitsingh99): Remove NaN Points

  // Sort based on z-axis value.
  sort(
    input_cloud->points.begin(), input_cloud->end(), [&](PointT & p1, PointT & p2) {
      return p1.z < p2.z;
    });

  // Extract initial ground seeds
  extract_initial_seeds(*input_cloud);

  // Iteratively fit and refine ground plane
  for (unsigned int i = 0; i < num_iterations; i++) {
    estimate_plane(*input_cloud);
    input_cloud->clear();
    g_not_ground_pc.clear();

    // Convert pointcloud to matrix
    Eigen::MatrixXf points_matrix(input_cloud_original.points.size(), 3);

    for (pcl::uindex_t j = 0; j < input_cloud_original.points.size(); ++j) {
      const auto & p = input_cloud_original.points[j];
      points_matrix.row(j) << p.x, p.y, p.z;
    }

    // Ground plane model
    Eigen::VectorXf result = points_matrix * model_.normal;

    // Threshold filter
    for (pcl::uindex_t r = 0; r < result.rows(); r++) {
      if (result[r] < model_.dist_threshold) {
        // Ground plane points
        input_cloud->points.push_back(input_cloud_original[r]);
      } else {
        g_not_ground_pc.points.push_back(input_cloud_original[r]);
      }
    }

    if (i == num_iterations - 1 && negative) {
      for (pcl::uindex_t r = 0; r < result.rows(); r++) {
        if (result[r] >= model_.dist_threshold) {
          // Non-Ground plane points
          g_not_ground_pc.points.push_back(input_cloud_original[r]);
        }
      }
      copyPointCloud(g_not_ground_pc, *input_cloud);
      break;
    }
  }

  return input_cloud;
}

template<typename PointT>
void GroundPlaneFittingImpl<PointT>::estimate_plane(const PointCloud & seed_points)
{
  Eigen::Matrix3f cov;
  Eigen::Vector4f pc_mean;
  pcl::computeMeanAndCovarianceMatrix(seed_points, cov, pc_mean);

  // Singular Value Decomposition
  Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov, Eigen::DecompositionOptions::ComputeFullU);

  // Use the least singular vector as normal
  model_.normal = (svd.matrixU().col(2));

  // Mean ground seeds value
  Eigen::Vector3f seeds_mean = pc_mean.head<3>();

  // According to normal.T * [x,y,z] = -d
  model_.d = -(model_.normal.transpose() * seeds_mean)(0, 0);

  // Set model distance threshold to plane_dist_threshold - d
  model_.dist_threshold = plane_dist_threshold - model_.d;
}

template<typename PointT>
void GroundPlaneFittingImpl<PointT>::extract_initial_seeds(PointCloud & sorted_points)
{
  // LPR is the mean of low point representative
  double sum = 0;
  pcl::uindex_t i = 0;

  // Calculate the mean height value
  for (; i < num_lpr && i < sorted_points.points.size(); ++i) {
    sum += sorted_points.points[i].z;
  }

  double lpr_height = i != 0 ? sum / i : 0;

  // Remove points having height greater than lpr_height + initial_seeds_threshold
  sorted_points.points.erase(
    std::remove_if(
      sorted_points.points.begin(), sorted_points.points.end(), [&](const auto p) {
        return p.z > lpr_height + initial_seeds_threshold;
      }), sorted_points.points.end());
}
}  // namespace impl


template<typename PointT>
typename GroundPlaneFitting<PointT>::PointCloudPtr GroundPlaneFitting<PointT>::process(
  GroundPlaneFitting<PointT>::PointCloudPtr cloud_in)
{
  cloud_in = processor_.fit(cloud_in);
  return cloud_in;
}

template<typename PointT>
void GroundPlaneFitting<PointT>::declare_parameters(rclcpp::Node::SharedPtr & node)
{
  param_callback_ = node->add_on_set_parameters_callback(
    [&](auto && arg)
    {
      return set_parameters(std::forward<decltype(arg)>(arg));
    });

  node->declare_parameter<int64_t>(this->plugin_name_ + ".num_iterations", 10);
  node->declare_parameter<int64_t>(this->plugin_name_ + ".num_lpr", 20);
  node->declare_parameter<double>(this->plugin_name_ + ".initial_seeds_threshold", 1.0);
  node->declare_parameter<double>(this->plugin_name_ + ".plane_dist_threshold", 0.25);
  node->declare_parameter<bool>(this->plugin_name_ + ".negative", false);
}

template<typename PointT>
rcl_interfaces::msg::SetParametersResult
GroundPlaneFitting<PointT>::set_parameters(const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & parameter : parameters) {
    // TODO(shrijitsingh99): Add try-catch for catching any parameter exceptions and set
    // parameter result to false
    const std::string & parameter_name = parameter.get_name();
    if (parameter_name == this->plugin_name_ + ".num_iterations") {
      processor_.num_iterations = static_cast<unsigned>(std::max(0l, parameter.as_int()));
    } else if (parameter_name == this->plugin_name_ + ".num_lpr") {
      processor_.num_lpr = static_cast<unsigned>(std::max(0l, parameter.as_int()));
    } else if (parameter_name == this->plugin_name_ + ".initial_seeds_threshold") {
      processor_.initial_seeds_threshold = parameter.as_double();
    } else if (parameter_name == this->plugin_name_ + ".plane_dist_threshold") {
      processor_.plane_dist_threshold = parameter.as_double();
    } else if (parameter_name == this->plugin_name_ + ".negative") {
      processor_.negative = parameter.as_bool();
    }
  }
  return result;
}
}  // namespace pcl_processor

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  pcl_processor::GroundPlaneFitting<pcl::PointXYZ>,
  pcl_processor::PointCloudProcessor<pcl::PointXYZ>)
