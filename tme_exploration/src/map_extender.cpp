/// \file
/// \brief This node reads in the robot's local SLAM map and creates a larger map to be used for the map_merge node
///
/// PUBLISHES:
///     new_map (nav_msgs/OccupancyGrid): Publishes an expanded map with new width, height and origin
/// SUBSCRIBES:
///     map (nav_msgs/OccupancyGrid): Reads the map created by SLAM

#include "ros/ros.h"
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include <std_msgs/Header.h>
#include <nav_msgs/MapMetaData.h>

// Define global message 
nav_msgs::OccupancyGrid slam_map;

/// \brief Reads the map data published from slam_toolbox
/// \param msg - map message
/// \returns nothing
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr & msg)
{
  slam_map.header = msg->header;
  slam_map.info = msg->info;
  slam_map.data = msg->data;
  std::cout << "Got to map callback" << std::endl;
}

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "map_expansion_node");
  ros::NodeHandle nh;

  // Create the publisher and subscriber
  const auto new_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("new_map", 100);
  const auto map_meta_sub = nh.subscribe<nav_msgs::OccupancyGrid>("map", 100, mapCallback);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    if ( (slam_map.data.size() != 0) || slam_map.info.origin.position.x !=0 )
    {
      const size_t sizing_ = 1000;

      const size_t width_ = sizing_;
      const size_t height_ = sizing_;

      // Create the new larger maps for each robot
      nav_msgs::OccupancyGrid new_map;
      new_map.header.frame_id = "new_map";
      new_map.info.resolution = 0.05;
      new_map.info.origin.position.x = -(width_ * new_map.info.resolution * 0.5); //size * resolution * 0.5 for middle point
      new_map.info.origin.position.y = -(height_ * new_map.info.resolution * 0.5);
      new_map.info.origin.position.z = 0.0;
      new_map.info.origin.orientation.w = 0.0;

      // Set the new map width and heights 
      new_map.info.width = width_;
      new_map.info.height = height_;
      // Determine how much space to fill in with unknown cells bewteen bottom of the new sized map and the original slam map
      const size_t bottom_width_ = (slam_map.info.origin.position.x - new_map.info.origin.position.x) / new_map.info.resolution;

      const size_t bottom_height_ = (slam_map.info.origin.position.y - new_map.info.origin.position.y) / new_map.info.resolution;

      // Map starts loading in from origin, which is the bottom right corner (for the deault orientation in rviz)
      // From the origin, the row components corresponds with width (+ x-dir which is up and + y-dir is to the left)
      // Fill in the all new cells in the new map with unknowns (-1)

      int c0 = 0; // start a counter for map0

      // Fill in the space between start of the new map to the start of the local SLAM map with -1s
      // (for the default orientation in Rviz, this is the space to the right of the SLAM map)
      for (int i=0;  i < new_map.info.width * bottom_height_; i++)
      {
        new_map.data.push_back(-1);
      }

      // Fill in the spaces on either side of the local SLAM map with -1s, but dont replace the current values from the local SLAM map
      for (int item_counter=0; item_counter < slam_map.info.height; item_counter++)
      {
        // For all new cells between the new starting width and the original SLAM starting width, fill with -1s
        // (for the default orientation in Rviz, this is the space below the SLAM map)
        for (int q=0; q < bottom_width_; q++)
        {
          new_map.data.push_back(-1);
        }

        // Fill in the current SLAM map information, in its initial location
        for (int a = 0; a < slam_map.info.width; a++)
        {
          new_map.data.push_back(slam_map.data[c0]);
          c0++;
        }

        // For all new cells between the new ending width and the original SLAM end width, fill with -1s
        // (for the default orientation in Rviz, this is the space above the SLAM map)
        for (int u=0; u < (new_map.info.width - slam_map.info.width - bottom_width_); u++)
        {
          new_map.data.push_back(-1);
        }
      } 

      // Fill in the space between the end of the original SLAM map to the end of the new map with -1s
      // (for the default orientation in Rviz, this is the space to the left of the SLAM map)
      for (int z=0;  z < ((height_ - slam_map.info.height - bottom_height_) * new_map.info.width); z++)
      {
        new_map.data.push_back(-1);
      }

      // Publish the new map
      new_map_pub.publish(new_map);
    }

      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;
}