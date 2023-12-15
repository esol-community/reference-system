// Copyright 2021 Robert Bosch GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <string>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <set>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "rcpputils/thread.hpp"
#include "rcl/context.h"
#include "rcutils/thread_attr.h"

#include "reference_system/system/type/rclcpp_system.hpp"

#include "autoware_reference_system/autoware_system_builder.hpp"
#include "autoware_reference_system/system/timing/benchmark.hpp"
#include "autoware_reference_system/system/timing/default.hpp"
#include "autoware_reference_system/priorities.hpp"

#ifdef USE_USER_IMPLEMENTED_ATTR_FIND_ROUTINE
std::optional<rcpputils::ThreadAttribute> find_thread_attr(const std::string & name)
{
  rcl_context_t const * ctx = rclcpp::contexts::get_global_default_context()->get_rcl_context().get();
  rcutils_thread_attrs_t * rcl_attrs = rcl_context_get_thread_attrs(ctx);
  
  if (!rcl_attrs) {
    return std::nullopt;
  }

  for (std::size_t i = 0; i < rcl_attrs->num_attributes; ++i) {
    if (name == rcl_attrs->attributes[i].name) {
      rcutils_thread_attr_t const & rcl_attr = rcl_attrs->attributes[i];
      rcpputils::ThreadAttribute attr(rcl_attr);
      return {attr};
    }
  }
  
  return std::nullopt;
}
#endif // USE_USER_IMPLEMENTED_ATTR_FIND_ROUTINE

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  using TimeConfig = nodes::timing::Default;
  // uncomment for benchmarking
  // using TimeConfig = nodes::timing::BenchmarkCPUUsage;
  // set_benchmark_mode(true);

  auto nodes_vec = create_autoware_nodes<RclcppSystem, TimeConfig>();
  using NodeMap = std::unordered_map<std::string,
      std::shared_ptr<RclcppSystem::NodeBaseType>>;

  NodeMap nodes;
  for (const auto & node : nodes_vec) {
    nodes.emplace(node->get_name(), node);
    std::cout << node->get_name() << "\n";
  }


#ifdef USE_USER_IMPLEMENTED_ATTR_FIND_ROUTINE
  auto logger = rclcpp::get_logger("autoware_default_prioritized");
  auto hotpath_attr = find_thread_attr("RCLCPP_EXECUTOR_HOTPATH");
  if (!hotpath_attr) {
    RCLCPP_ERROR(logger, "not found thread attribute \"RCLCPP_EXECUTOR_HOTPATH\"");
    return 1;
  }
  auto planner_attr= find_thread_attr("RCLCPP_EXECUTOR_PLANNER");
  if (!planner_attr) {
    RCLCPP_ERROR(logger, "not found thread attribute \"RCLCPP_EXECUTOR_PLANNER\"");
    return 1;
  }
  
  rclcpp::executors::SingleThreadedExecutor front_exe(rclcpp::ExecutorOptions(), *hotpath_attr);
  rclcpp::executors::SingleThreadedExecutor rear_exe(rclcpp::ExecutorOptions(), *hotpath_attr);
  rclcpp::executors::SingleThreadedExecutor fusion_exe(rclcpp::ExecutorOptions(), *hotpath_attr);
  rclcpp::executors::SingleThreadedExecutor planner_exe(rclcpp::ExecutorOptions(), *planner_attr);
#else // USE_USER_IMPLEMENTED_ATTR_FIND_ROUTINE
  auto front_exe_options = rclcpp::ExecutorOptions();
  auto rear_exe_options = rclcpp::ExecutorOptions();
  auto fusion_exe_options = rclcpp::ExecutorOptions();
  auto planner_exe_options = rclcpp::ExecutorOptions();
  front_exe_options.name = "RCLCPP_EXECUTOR_HOTPATH";
  rear_exe_options.name = "RCLCPP_EXECUTOR_HOTPATH";
  fusion_exe_options.name = "RCLCPP_EXECUTOR_HOTPATH";
  planner_exe_options.name = "RCLCPP_EXECUTOR_PLANNER";

  rclcpp::executors::SingleThreadedExecutor front_exe(front_exe_options);
  rclcpp::executors::SingleThreadedExecutor rear_exe(rear_exe_options);
  rclcpp::executors::SingleThreadedExecutor fusion_exe(fusion_exe_options);
  rclcpp::executors::SingleThreadedExecutor planner_exe(planner_exe_options);
#endif // USE_USER_IMPLEMENTED_ATTR_FIND_ROUTINE
  rclcpp::executors::SingleThreadedExecutor other_exe;
  
  std::set<std::string> front_nodes = {"FrontLidarDriver", "PointsTransformerFront"};
  std::set<std::string> rear_nodes = {"RearLidarDriver", "PointsTransformerRear"};
  std::set<std::string> fusion_nodes = {"PointCloudFusion",
    "RayGroundFilter",
    "EuclideanClusterDetector",
    "ObjectCollisionEstimator"};
  std::set<std::string> planner_nodes = {"BehaviorPlanner"};
  std::set<std::string> other_nodes = {"PointCloudMap",
    "Visualizer",
    "Lanelet2Map",
    "EuclideanClusterSettings",
    "PointCloudMapLoader",
    "MPCController",
    "VehicleInterface",
    "VehicleDBWSystem",
    "NDTLocalizer",
    "Lanelet2GlobalPlanner",
    "Lanelet2MapLoader",
    "ParkingPlanner",
    "LanePlanner",
    "IntersectionOutput",
    "VoxelGridDownsampler"};

  for (const auto & node : front_nodes) {
    front_exe.add_node(nodes.at(node));
  }
  for (const auto & node : rear_nodes) {
    rear_exe.add_node(nodes.at(node));
  }
  for (const auto & node : fusion_nodes) {
    std::cout << node << "\n";
    fusion_exe.add_node(nodes.at(node));
  }
  for (const auto & node : planner_nodes) {
    planner_exe.add_node(nodes.at(node));
  }
  for (const auto & node : other_nodes) {
    other_exe.add_node(nodes.at(node));
  }

  rcpputils::Thread front_thread {[&]() {
      front_exe.spin();
    }};
  rcpputils::Thread rear_thread {[&]() {
      rear_exe.spin();
    }};
  rcpputils::Thread fusion_thread {[&]() {
      fusion_exe.spin();
    }};
  rcpputils::Thread planner_thread {[&]() {
      planner_exe.spin();
    }};
  rcpputils::Thread other_thread {[&]() {
      other_exe.spin();
    }};

  front_thread.join();
  rear_thread.join();
  fusion_thread.join();
  planner_thread.join();

  rclcpp::shutdown();
}
