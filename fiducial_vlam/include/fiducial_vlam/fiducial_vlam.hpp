#pragma once

#include <memory>

namespace rclcpp
{
  class Node; //
  class NodeOptions; //
}

namespace fiducial_vlam
{
  std::shared_ptr<rclcpp::Node> vdet_node_factory(const rclcpp::NodeOptions &options); //
  std::shared_ptr<rclcpp::Node> vloc_node_factory(const rclcpp::NodeOptions &options); //
  std::shared_ptr<rclcpp::Node> vlocx_node_factory(const rclcpp::NodeOptions &options); //
  std::shared_ptr<rclcpp::Node> vmap_node_factory(const rclcpp::NodeOptions &options); //
}
