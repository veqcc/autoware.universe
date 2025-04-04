// Copyright 2021 Tier IV, Inc.
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

#include "scene.hpp"

#include <autoware/motion_utils/marker/marker_helper.hpp>
#include <autoware/motion_utils/marker/virtual_wall_marker_creator.hpp>
#include <autoware_utils/math/constants.hpp>
#include <autoware_utils/ros/marker_helper.hpp>

#include <vector>
using autoware::motion_utils::createStopVirtualWallMarker;
using autoware_utils::append_marker_array;
using autoware_utils::create_default_marker;
using autoware_utils::create_marker_color;
using autoware_utils::create_marker_orientation;
using autoware_utils::create_marker_position;
using autoware_utils::create_marker_scale;
using autoware_utils::to_msg;
using namespace std::literals::string_literals;

namespace autoware::behavior_velocity_planner
{

autoware::motion_utils::VirtualWalls VirtualTrafficLightModule::createVirtualWalls()
{
  autoware::motion_utils::VirtualWalls virtual_walls;
  autoware::motion_utils::VirtualWall wall;
  wall.text = "virtual_traffic_light";
  wall.ns = std::to_string(module_id_) + "_";
  wall.style = autoware::motion_utils::VirtualWallType::stop;
  const auto & d = module_data_;
  // virtual_wall_stop_line
  std::vector<geometry_msgs::msg::Pose> wall_poses;
  if (d.stop_head_pose_at_stop_line) wall_poses.push_back(*d.stop_head_pose_at_stop_line);
  // virtual_wall_end_line
  if (d.stop_head_pose_at_end_line) wall_poses.push_back(*d.stop_head_pose_at_end_line);
  for (const auto & p : wall_poses) {
    wall.pose = p;
    virtual_walls.push_back(wall);
  }

  return virtual_walls;
}

visualization_msgs::msg::MarkerArray VirtualTrafficLightModule::createDebugMarkerArray()
{
  visualization_msgs::msg::MarkerArray debug_marker_array;

  // Common
  const auto & m = map_data_;
  const auto now = clock_->now();

  // instrument_id
  {
    auto marker = create_default_marker(
      "map", now, "instrument_id", module_id_, visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
      create_marker_scale(0.0, 0.0, 1.0), create_marker_color(1.0, 1.0, 1.0, 0.999));

    marker.pose.position = to_msg(m.instrument_center);
    marker.text = m.instrument_id;

    debug_marker_array.markers.push_back(marker);
  }

  // instrument_center
  {
    auto marker = create_default_marker(
      "map", now, "instrument_center", module_id_, visualization_msgs::msg::Marker::SPHERE,
      create_marker_scale(0.3, 0.3, 0.3), create_marker_color(1.0, 0.0, 0.0, 0.999));

    marker.pose.position = to_msg(m.instrument_center);

    debug_marker_array.markers.push_back(marker);
  }

  // stop_line
  if (m.stop_line) {
    auto marker = create_default_marker(
      "map", now, "stop_line", module_id_, visualization_msgs::msg::Marker::LINE_STRIP,
      create_marker_scale(0.3, 0.0, 0.0), create_marker_color(1.0, 1.0, 1.0, 0.999));

    for (const auto & p : *m.stop_line) {
      marker.points.push_back(to_msg(p));
    }

    debug_marker_array.markers.push_back(marker);
  }

  // start_line
  {
    auto marker = create_default_marker(
      "map", now, "start_line", module_id_, visualization_msgs::msg::Marker::LINE_STRIP,
      create_marker_scale(0.3, 0.0, 0.0), create_marker_color(0.0, 1.0, 0.0, 0.999));

    for (const auto & p : m.start_line) {
      marker.points.push_back(to_msg(p));
    }

    debug_marker_array.markers.push_back(marker);
  }

  // end_lines
  {
    auto marker = create_default_marker(
      "map", now, "end_lines", module_id_, visualization_msgs::msg::Marker::LINE_LIST,
      create_marker_scale(0.3, 0.0, 0.0), create_marker_color(0.0, 1.0, 1.0, 0.999));

    for (const auto & line : m.end_lines) {
      for (size_t i = 1; i < line.size(); ++i) {
        marker.points.push_back(to_msg(line.at(i - 1)));
        marker.points.push_back(to_msg(line.at(i)));
      }
    }

    debug_marker_array.markers.push_back(marker);
  }

  return debug_marker_array;
}
}  // namespace autoware::behavior_velocity_planner
