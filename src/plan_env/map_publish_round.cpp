// map_publish.cpp
#include "hybrid_a_star/map_publish.h"

MapPublisher::MapPublisher() {
  grid_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1);
  obstacle_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/obstacle_points", 1); // 初始化 PointCloud2 发布器
  center_line_pub_ = nh_.advertise<nav_msgs::Path>("/center_line", 1);
  grid_resolution_ = 1.0; // 10 cm resolution
  map_frame_ = "map";
}

void MapPublisher::publishComplexMap() {
  int width = 100;  // Larger map width
  int height = 100; // Larger map height
  nav_msgs::OccupancyGrid grid_msg;
  grid_msg.header.frame_id = map_frame_;
  grid_msg.header.stamp = ros::Time::now();
  grid_msg.info.resolution = grid_resolution_;
  grid_msg.info.width = width;
  grid_msg.info.height = height;
  grid_msg.info.origin.position.x = 0.0;
  grid_msg.info.origin.position.y = 0.0;
  grid_msg.info.origin.orientation.w = 1.0;
  grid_msg.data.resize(width * height);

  // Initialize map with free space
  std::fill(grid_msg.data.begin(), grid_msg.data.end(), 0);

  // Add fixed rectangular obstacles
  // drawRectangle(grid_msg, 6, 50, 11, 4);
  // drawRectangle(grid_msg, 20, 35, 6, 40);
  // drawRectangle(grid_msg, 48, 15, 8, 20);
  // drawRectangle(grid_msg, 20, 20, 2, 2);

  grid_map_pub_.publish(grid_msg);
}
// 发布障碍点云
void addStraightLinePoints(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double start_x, double end_x, double y, double step) {
  for (double x = start_x; x <= end_x; x += step) {
      cloud->points.emplace_back(x, y, 0.0);
  }
}
void addArcPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double center_x, double center_y, 
                             double radius, double start_theta, double end_theta, double step) {
  for (double theta = start_theta; theta >= end_theta; theta -= step) {
      double x = center_x + radius * cos(theta);
      double y = center_y + radius * sin(theta);
      cloud->points.emplace_back(x, y, 0.0);
  }
}
// 发布道路中心线
void MapPublisher::addPathPoints(nav_msgs::Path& path, double start_x, double end_x, double y, double step, const ros::Time& stamp) {
  for (double x = start_x; x <= end_x; x += step) {
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = map_frame_;
      pose.header.stamp = stamp;
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0; // 默认朝向
      path.poses.push_back(pose);
  }
}
void MapPublisher::addPathArcPoints(nav_msgs::Path& path, double center_x, double center_y, 
  double radius, double start_theta, double end_theta, double step, const ros::Time& stamp) {
  for (double theta = start_theta; theta >= end_theta; theta -= step) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = map_frame_;
    pose.header.stamp = stamp;
    pose.pose.position.x = center_x + radius * cos(theta);
    pose.pose.position.y = center_y + radius * sin(theta);
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0; // 默认朝向
    path.poses.push_back(pose);
  }
}
void MapPublisher::publishObstaclePoints() {
  // 定义操场几何参数
  const double center_x = 50.0;  // 中心点 x 坐标
  const double center_y = 20.0;   // 中心点 y 坐标
  const double track_length = 60.0;  // 直道长度
  const double track_width = 6.0;    // 直道宽度
  const double inner_radius = 5.0;   // 内弯道半径

  double outer_radius = inner_radius + track_width;  // 外弯道半径

  double right_center_x = center_x + track_length / 2;     // 右半圆中心 x 坐标
  double right_center_y = center_y;                        // 半圆中心 y 坐标
  double left_center_x = center_x - track_length / 2;      // 左半圆中心 x 坐标
  double left_center_y = center_y;                         // 半圆中心 y 坐标
  double step_straight = 0.5;  // 直道点间距
  double step_inner_arc = 0.1; // 内弯道点间距
  double step_outer_arc = 0.05;// 外弯道点间距

  // step1：创建 PCL 点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // 添加直道点
  addStraightLinePoints(cloud, left_center_x, right_center_x, center_y + inner_radius, step_straight);
  addStraightLinePoints(cloud, left_center_x, right_center_x, center_y + outer_radius, step_straight);
  addStraightLinePoints(cloud, left_center_x, right_center_x, center_y - inner_radius, step_straight);
  addStraightLinePoints(cloud, left_center_x, right_center_x, center_y - outer_radius, step_straight);

  // 添加右半圆点
  addArcPoints(cloud, right_center_x, right_center_y, inner_radius, M_PI / 2, -M_PI / 2, step_inner_arc);
  addArcPoints(cloud, right_center_x, right_center_y, outer_radius, M_PI / 2, -M_PI / 2, step_outer_arc);

  // 添加左半圆点
  addArcPoints(cloud, left_center_x, left_center_y, inner_radius, M_PI, M_PI / 2, step_inner_arc);
  addArcPoints(cloud, left_center_x, left_center_y, inner_radius, -M_PI / 2, -M_PI, step_inner_arc);
  addArcPoints(cloud, left_center_x, left_center_y, outer_radius, M_PI, M_PI / 2, step_outer_arc);
  addArcPoints(cloud, left_center_x, left_center_y, outer_radius, -M_PI / 2, -M_PI, step_outer_arc);

  // cloud->points.emplace_back(50.0, 26.0, 0.0);
  // cloud->points.emplace_back(50.0, 27.0, 0.0);
  cloud->points.emplace_back(50.0, 28.0, 0.0);

  // cloud->points.emplace_back(40.0, 28.0, 0.0);
  // cloud->points.emplace_back(40.0, 29.0, 0.0);
  cloud->points.emplace_back(40.0, 30.0, 0.0);

  cloud->points.emplace_back(70.0, 28.0, 0.0);
  cloud->points.emplace_back(70.0, 29.0, 0.0);


  // 设置点云属性
  cloud->width = cloud->points.size();
  cloud->height = 1;
  cloud->is_dense = true;

  // 转换为 ROS PointCloud2 消息
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud, cloud_msg);
  cloud_msg.header.frame_id = map_frame_;
  cloud_msg.header.stamp = ros::Time::now();
  // 发布点云
  obstacle_points_pub_.publish(cloud_msg);

  // step2：创建中心线路径
  nav_msgs::Path center_line_path;
  center_line_path.header.frame_id = map_frame_;
  center_line_path.header.stamp = ros::Time::now();

  // 添加直道路径点
  addPathPoints(center_line_path, left_center_x, right_center_x, 
                center_y + inner_radius + track_width / 2, step_straight, center_line_path.header.stamp);
  // 添加右半圆路径点
  addPathArcPoints(center_line_path, right_center_x, right_center_y, 
    inner_radius + track_width / 2, M_PI / 2, -M_PI / 2, step_inner_arc, center_line_path.header.stamp);

  addPathPoints(center_line_path, right_center_x, left_center_x, 
                center_y - inner_radius - track_width / 2, -step_straight, center_line_path.header.stamp);

  // 添加左半圆路径点
  addPathArcPoints(center_line_path, left_center_y, left_center_y, inner_radius + track_width / 2, -M_PI / 2, -M_PI, step_inner_arc, center_line_path.header.stamp);
  addPathArcPoints(center_line_path, left_center_x, left_center_x, inner_radius + track_width / 2, M_PI, M_PI / 2, step_inner_arc, center_line_path.header.stamp);
  
  // 发布中心线路径
  center_line_pub_.publish(center_line_path);
}

void MapPublisher::drawRectangle(nav_msgs::OccupancyGrid& grid, int x, int y, int width, int height) {
  for (int i = x; i < x + width && i < static_cast<int>(grid.info.width); ++i) {
    for (int j = y; j < y + height && j < static_cast<int>(grid.info.height); ++j) {
      if (i >= 0 && j >= 0) grid.data[i + j * grid.info.width] = 100;
    }
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "map_publisher_node");
  MapPublisher map_pub;
  ros::Rate rate(1); // Publish every 1 second
  while (ros::ok()) {
    map_pub.publishComplexMap();
    map_pub.publishObstaclePoints(); // 发布点云
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}