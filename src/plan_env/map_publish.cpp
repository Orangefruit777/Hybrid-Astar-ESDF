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

void MapPublisher::initializeObstacleCloud() {
  // 创建 PCL 点云
  obstacle_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  double obstacle_interval = 0.5;

  // int num_obstacles = 150;
  // double map_size = 50.0;
  // // 在固定范围内随机生成障碍物点
  // for (int i = 0; i < num_obstacles; ++i) {
  //   double x = static_cast<double>(rand()) / RAND_MAX * map_size;
  //   double y = static_cast<double>(rand()) / RAND_MAX * map_size;
  //   obstacle_cloud_->points.emplace_back(x, y, 0.0);
  // }

  // 添加矩形范围点云障碍物
  for (double x = 18.0; x <= 20.0; x += obstacle_interval) {
    for (double y = 20.0; y <= 25.0; y += obstacle_interval) {
      obstacle_cloud_->points.emplace_back(x, y, 0.0);
    }
  }
  for (double x = 15.0; x <= 25.0; x += obstacle_interval) {
    for (double y = 28.0; y <= 30.0; y += obstacle_interval) {
      obstacle_cloud_->points.emplace_back(x, y, 0.0);
    }
  }


  // 添加固定的障碍物点
  obstacle_cloud_->points.emplace_back(20.0, 10.0, 0.0);
  obstacle_cloud_->points.emplace_back(20.0, 11.0, 0.0);
  // obstacle_cloud_->points.emplace_back(5.0, 15.0, 0.0);
  // obstacle_cloud_->points.emplace_back(5.0, 26.0, 0.0);

  for (double x = 10.0; x <= 30.0; x += obstacle_interval) {
    obstacle_cloud_->points.emplace_back(x, 9.0, 0.0);
    obstacle_cloud_->points.emplace_back(x, 13.5, 0.0);
  }

  for (double x = 24.0; x <= 26.0; x += obstacle_interval) {
    for (double y = 35.0; y <= 40.0; y += obstacle_interval){
      obstacle_cloud_->points.emplace_back(x, y, 0.0);
    }
  }

  // 添加车库形状障碍物点
  // 直角车库
  for(double x = 35.0; x <= 38.0; x += obstacle_interval){
    obstacle_cloud_->points.emplace_back(x, 30.0, 0.0);
    obstacle_cloud_->points.emplace_back(x, 28.0, 0.0);
  }
  obstacle_cloud_->points.emplace_back(38.0, 28.5, 0.0);
  obstacle_cloud_->points.emplace_back(38.0, 29.0, 0.0);
  obstacle_cloud_->points.emplace_back(38.0, 29.5, 0.0);

  // 短直角车库
  for(double x = 36.0; x <= 38.0; x += obstacle_interval){
    obstacle_cloud_->points.emplace_back(x, 35.0, 0.0);
    obstacle_cloud_->points.emplace_back(x, 37.0, 0.0);
  }
  obstacle_cloud_->points.emplace_back(38.0, 35.5, 0.0);
  obstacle_cloud_->points.emplace_back(38.0, 36.0, 0.0);
  obstacle_cloud_->points.emplace_back(38.0, 36.5, 0.0);

  // 平行车库
  double right_x = 36.5;
  for(double x = 35.0; x <= 36.5; x += obstacle_interval){
    obstacle_cloud_->points.emplace_back(x, 20.0, 0.0);
    obstacle_cloud_->points.emplace_back(x, 23.0, 0.0);
  }
  obstacle_cloud_->points.emplace_back(right_x, 20.5, 0.0);
  obstacle_cloud_->points.emplace_back(right_x, 21.0, 0.0);
  obstacle_cloud_->points.emplace_back(right_x, 21.5, 0.0);
  obstacle_cloud_->points.emplace_back(right_x, 22.0, 0.0);
  obstacle_cloud_->points.emplace_back(right_x, 22.5, 0.0);

  // 斜角车库
  for(double i = 0; i <= 2.0; i += obstacle_interval){
    obstacle_cloud_->points.emplace_back(36.5 + i, 13.5 - i, 0.0);
  }
  for(double i = 0; i <= 2.0; i += obstacle_interval){
    obstacle_cloud_->points.emplace_back(35.0 + i, 12.0 - i, 0.0);
  }
  obstacle_cloud_->points.emplace_back(37.5, 10.5, 0.0);
  obstacle_cloud_->points.emplace_back(38.0, 11.0, 0.0);

  for (double y = 8.0; y <= 38.0; y += obstacle_interval) {
    obstacle_cloud_->points.emplace_back(32.0, y, 0.0);
  }

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

// void MapPublisher::publishLanePoints() {
//   nav_msgs::Path lane_path;
//   lane_path.header.frame_id = map_frame_;
//   lane_path.header.stamp = ros::Time::now();
//   std::vector<Eigen::Vector2d> lane_points;
//   for(double i = 5.0; i < 45.0; i += 0.5){
//     lane_points.emplace_back(i, 20.0);
//   }

//   // 填充路径点
//   for (const auto& point : lane_points) {
//     geometry_msgs::PoseStamped pose;
//     pose.header.frame_id = map_frame_;
//     pose.header.stamp = ros::Time::now();
//     pose.pose.position.x = point.x();
//     pose.pose.position.y = point.y();
//     pose.pose.position.z = 0.0;
//     pose.pose.orientation.w = 1.0; // 默认朝向
//     lane_path.poses.push_back(pose);
//   }

//   // 发布参考线
//   lane_points_pub_.publish(lane_path);
// }

void MapPublisher::publishLanePoints() {
  nav_msgs::Path lane_path;
  std::vector<Eigen::Vector2d> lane_points;
  for(double i = 5.0; i < 45.0; i += 0.5){
    lane_points.emplace_back(i, 40.0);
  }

  geometry_msgs::PoseStamped pose_stamped;
  for (const auto &pose: lane_points) {
    pose_stamped.header.frame_id = map_frame_;
    pose_stamped.pose.position.x = pose.x();
    pose_stamped.pose.position.y = pose.y();
    pose_stamped.pose.position.z = 0.0;
    pose_stamped.pose.orientation.w = 1.0;

    lane_path.poses.emplace_back(pose_stamped);
  }

  lane_path.header.frame_id = map_frame_;
  lane_path.header.stamp = ros::Time::now();

  lane_points_pub_.publish(lane_path);
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "map_publisher_node");
  MapPublisher map_pub;
  ros::Rate rate(1); // Publish every 1 second
  while (ros::ok()) {
    map_pub.publishMap();
    map_pub.publishObstaclePoints(); // 发布点云
    map_pub.publishLanePoints(); 
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}