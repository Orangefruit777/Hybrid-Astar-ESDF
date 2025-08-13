// map_publish.h
#ifndef MAP_PUBLISHER_H
#define MAP_PUBLISHER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h> // 使用 PointCloud2
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>

class MapPublisher {
public:
  MapPublisher();
  void publishComplexMap();
  void publishObstaclePoints(); // 发布点云
  void addPathPoints(nav_msgs::Path& path, double start_x, double end_x, double y, double step, const ros::Time& stamp);
  void addPathArcPoints(nav_msgs::Path& path, double center_x, double center_y, 
    double radius, double start_theta, double end_theta, double step, const ros::Time& stamp);
private:
  void drawRectangle(nav_msgs::OccupancyGrid& grid, int x, int y, int width, int height);
  ros::NodeHandle nh_;
  ros::Publisher grid_map_pub_;
  ros::Publisher obstacle_points_pub_; // 发布 PointCloud2
  ros::Publisher center_line_pub_;
  double grid_resolution_;
  std::string map_frame_;
};

#endif