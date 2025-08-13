#include "plan_env/map_publish.h"

MapPublisher::MapPublisher() {
  grid_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1);
  obstacle_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/obstacle_points", 1); // 初始化 PointCloud2 发布器
  lane_points_pub_ = nh_.advertise<nav_msgs::Path>("/lane_points", 1);
  grid_resolution_ = 0.2; // 10 cm resolution
  map_frame_ = "world";
  // Initialize obstacle point cloud
  initializeObstacleCloud();
}

// 添加矩形障碍物
void addObstacleRectangle(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double x_min, double x_max, double y_min, double y_max, double step) {
  for (double x = x_min; x <= x_max; x += step) {
      for (double y = y_min; y <= y_max; y += step) {
          cloud->points.emplace_back(x, y, 0.0);
      }
  }
}

void MapPublisher::initializeObstacleCloud() {
  // 创建 PCL 点云
  obstacle_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  double obstacle_interval = 0.2;

  // 添加边界障碍物
  addObstacleRectangle(obstacle_cloud_, 0.0, 50.0, 0.0, 1.0, obstacle_interval);
  addObstacleRectangle(obstacle_cloud_, 0.0, 50.0, 49.0, 50.0, obstacle_interval);
  addObstacleRectangle(obstacle_cloud_, 0.0, 1.0, 1.0, 49.0, obstacle_interval);
  addObstacleRectangle(obstacle_cloud_, 49.0, 50.0, 1.0, 49.0, obstacle_interval);

  addObstacleRectangle(obstacle_cloud_, 15.0, 16.0, 25.0, 35.0, obstacle_interval);
  addObstacleRectangle(obstacle_cloud_, 35.0, 36.0, 12.0, 25.0, obstacle_interval);
  
  addObstacleRectangle(obstacle_cloud_, 18.0, 26.0, 15.0, 18.0, obstacle_interval);
  addObstacleRectangle(obstacle_cloud_, 20.0, 35.0, 35.0, 38.0, obstacle_interval);
  
  addObstacleRectangle(obstacle_cloud_, 25.0, 30.0, 20.0, 30.0, obstacle_interval);

  addObstacleRectangle(obstacle_cloud_, 5.0, 12.0, 18.0, 26.0, obstacle_interval);
  addObstacleRectangle(obstacle_cloud_, 41.0, 46.0, 22.0, 32.0, obstacle_interval);

  addObstacleRectangle(obstacle_cloud_, 19.0, 22.0, 3.0, 8.0, obstacle_interval);
  addObstacleRectangle(obstacle_cloud_, 26.0, 33.0, 6.0, 9.0, obstacle_interval);
  addObstacleRectangle(obstacle_cloud_, 19.0, 22.0, 40.0, 46.0, obstacle_interval);
  addObstacleRectangle(obstacle_cloud_, 28.0, 34.0, 40.0, 46.0, obstacle_interval);
  
  // 四个角落的简单障碍物
  addObstacleRectangle(obstacle_cloud_, 5.0, 15.0, 5.0, 10.0, obstacle_interval);
  addObstacleRectangle(obstacle_cloud_, 38.0, 45.0, 5.0, 8.0, obstacle_interval);
  addObstacleRectangle(obstacle_cloud_, 5.0, 10.0, 35.0, 43.0, obstacle_interval);
  addObstacleRectangle(obstacle_cloud_, 40.0, 45.0, 43.0, 46.0, obstacle_interval);

  

  // // 添加平行四边形障碍物
  // double y_offset = 0.0;
  // for (double x = 5.0; x <= 10.0; x += obstacle_interval) {
  //   for (double y = 20.0 + y_offset; y <= 25.0 + y_offset; y += obstacle_interval) {
  //     obstacle_cloud_->points.emplace_back(x, y, 0.0);
  //   }
  //   y_offset += obstacle_interval;
  // }

  // 设置点云属性
  obstacle_cloud_->width = obstacle_cloud_->points.size();
  obstacle_cloud_->height = 1;
  obstacle_cloud_->is_dense = true;
}

void MapPublisher::publishMap() {
  int width = 250;  // Larger map width
  int height = 250; // Larger map height
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
  grid_map_pub_.publish(grid_msg);
}

void MapPublisher::publishObstaclePoints() {
  // 转换为 ROS PointCloud2 消息
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*obstacle_cloud_, cloud_msg);
  cloud_msg.header.frame_id = map_frame_;
  cloud_msg.header.stamp = ros::Time::now();
  // 发布点云
  obstacle_points_pub_.publish(cloud_msg);
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "map_publisher_node");
  MapPublisher map_pub;
  ros::Rate rate(1); // Publish every 1 second
  while (ros::ok()) {
    map_pub.publishMap();
    map_pub.publishObstaclePoints(); // 发布点云
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}