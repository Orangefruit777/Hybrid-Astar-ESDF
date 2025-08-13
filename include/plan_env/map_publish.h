#ifndef MAP_PUBLISHER_H
#define MAP_PUBLISHER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>

class MapPublisher {
public:
  MapPublisher();
  void publishMap();
  void publishObstaclePoints();
  void publishLanePoints();

private:
  ros::NodeHandle nh_;
  ros::Publisher grid_map_pub_;
  ros::Publisher obstacle_points_pub_;
  ros::Publisher lane_points_pub_;
  double grid_resolution_;
  std::string map_frame_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud_; // Store fixed obstacle point cloud

  void initializeObstacleCloud(); // Initialize obstacle point cloud
};

#endif