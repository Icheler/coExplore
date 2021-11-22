/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <explore.h>

#include <thread>

#include <ros/console.h>

namespace exploration
{
Explore::Explore()
  : private_nh_("~")
  , costmap_client_(private_nh_, relative_nh_)
{
  double timeout;
  double min_frontier_size;
  last_markers_count_ = 0;
  private_nh_.param("planner_frequency", planner_frequency_, 1.0);
  private_nh_.param("min_frontier_size", min_frontier_size, 0.5);
  private_nh_.param("visualize", visualize_, true);

  frontiers_pub = private_nh_.advertise<tme_exploration::frontierArray>("frontierArray", 1000);
  search_ = exploration::FrontierSearch(costmap_client_.getCostmap(), min_frontier_size);
  
  position.x = 0.5;
  position.y = 0.5;
  
  
  if (visualize_) {
    marker_array_publisher_ = private_nh_.advertise<visualization_msgs::MarkerArray>("frontiers", 10);
  }
}

void Explore::Exploration(){
  auto costmap_msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map", relative_nh_);
  costmap_client_.updateFullMap(costmap_msg);
  std::vector<Frontier> frontiers = search_.searchFrom(position);
  ROS_WARN("We found %lu frontiers", frontiers.size());

  visualizeFrontiers(frontiers);
  frontiers_pub.publish(Explore::msgConversion(frontiers));
}

tme_exploration::frontierArray Explore::msgConversion(std::vector<Frontier> frontiers){
  tme_exploration::frontierArray msg;
  tme_exploration::frontier data;

  for(auto & elem: frontiers){
    data.size = elem.size;
    data.initial = elem.initial;
    data.centroid = elem.centroid;
    data.points = elem.points;
    msg.frontiers.push_back(data);
  }
  return msg;
}

void Explore::visualizeFrontiers(
    const std::vector<exploration::Frontier>& frontiers)
{
  std_msgs::ColorRGBA blue;
  blue.r = 0;
  blue.g = 0;
  blue.b = 1.0;
  blue.a = 1.0;
  std_msgs::ColorRGBA red;
  red.r = 1.0;
  red.g = 0;
  red.b = 0;
  red.a = 1.0;
  std_msgs::ColorRGBA green;
  green.r = 0;
  green.g = 1.0;
  green.b = 0;
  green.a = 1.0;

  ROS_WARN("visualising %lu frontiers", frontiers.size());
  visualization_msgs::MarkerArray markers_msg;
  std::vector<visualization_msgs::Marker>& markers = markers_msg.markers;
  visualization_msgs::Marker m;

  m.header.frame_id = costmap_client_.getGlobalFrameID();
  m.header.stamp = ros::Time::now();
  m.ns = "frontiers";
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = 0;
  m.color.g = 0;
  m.color.b = 255;
  m.color.a = 255;
  // lives forever
  m.lifetime = ros::Duration(0);
  m.frame_locked = true;

  // weighted frontiers are always sorted
  // double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;

  m.action = visualization_msgs::Marker::ADD;
  size_t id = 0;
  for (auto& frontier : frontiers) {
    m.type = visualization_msgs::Marker::POINTS;
    m.id = int(id);
    m.pose.position = {};
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.points = frontier.points;
    m.color = blue;
    markers.push_back(m);
    ++id;
    m.type = visualization_msgs::Marker::SPHERE;
    m.id = int(id);
    m.pose.position = frontier.initial;
    m.pose.orientation.w = 1.0;
    // scale frontier according to its cost (costier frontiers will be smaller)
    // double scale = std::min(std::abs(min_cost * 0.4 / frontier.cost), 0.5);
    /* m.scale.x = scale;
    m.scale.y = scale;
    m.scale.z = scale; */
    m.points = {};
    m.color = green;
    markers.push_back(m);
    ++id;
  }
  size_t current_markers_count = markers.size();

  // delete previous markers, which are now unused
  m.action = visualization_msgs::Marker::DELETE;
  for (; id < last_markers_count_; ++id) {
    m.id = int(id);
    ROS_WARN("print id: %i, %lu", m.id, last_markers_count_);
    markers.push_back(m);
  }
  last_markers_count_ = current_markers_count;
  marker_array_publisher_.publish(markers_msg);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tme_explore");
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                    ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }
  exploration::Explore explore;
  ros::Rate rate(1.);
  
  while(ros::ok()){
    explore.Exploration();
    rate.sleep();
  }

  return 0;
}
