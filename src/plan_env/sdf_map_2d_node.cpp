#include "plan_env/sdf_map_2d.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class SDFMap2DNode {
public:
  SDFMap2DNode(ros::NodeHandle& nh) : node_(nh){
    // Initialize map parameters
    is_map_initialized_ = false;
    size_ = Eigen::Vector2d(200, 200);
    resolution_ = 0.1;
    origin_ = Eigen::Vector2d(0.0, 0.0);

    mp_.frame_id_ = "world";
    // Initialize ROS subscribers and publishers
    goal_sub_ = node_.subscribe("/move_base_simple/goal", 1, &SDFMap2DNode::goalCallback, this);
    obstacle_sub_ = node_.subscribe("/obstacle_points", 1, &SDFMap2DNode::obstacleCallback, this);
  }    

private:
  void obstacleCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    if(is_map_initialized_){
      return;
    }
    // Convert PointCloud2 to PCL point cloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    // Extract obstacle points
    std::vector<Eigen::Vector2d> obstacles;
    for (const auto& point : cloud.points) {
      obstacles.emplace_back(point.x, point.y);
    }
    ros::Time start = ros::Time::now();
    sdf_map_.initMap(origin_, size_, resolution_, obstacles);
    ros::Time end = ros::Time::now();
    double dt = (end - start).toSec();
    ROS_INFO("Time to initialize map : %.2f ms", dt * 1000);
    is_map_initialized_ = true;
  }

  void goalCallback(const geometry_msgs::PoseStampedConstPtr& goal) {
    Eigen::Vector2d pos(goal->pose.position.x, goal->pose.position.y);
    ros::Time start = ros::Time::now();

    double dist = sdf_map_.getDistance(pos);
    Eigen::Vector2d grad;
    sdf_map_.evaluateEDTWithGrad(pos, dist, grad);

    ros::Time end = ros::Time::now();
    double dt = (end - start).toSec();

    ROS_INFO("Goal at (%.2f, %.2f)", pos(0), pos(1));
    ROS_INFO("Distance to nearest obs: %.2f m", dist);
    ROS_INFO("Gradient: (%.2f, %.2f)", grad(0), grad(1));
    ROS_INFO("Time to compute distance: %.2f ms", dt * 1000);
    ROS_INFO("------------------------------");
  }

  // void goalCallback(const geometry_msgs::PoseStampedConstPtr& goal) { 
  //   Eigen::Vector2d pos(goal->pose.position.x, goal->pose.position.y);
  //   ros::Time start = ros::Time::now();
  //   for(double i = 0; i < 30.0; i+=1.0){
  //     double x = pos(0) + 0.3 * i;
  //     double y = pos(1) + 0.3 * i;
  //     Eigen::Vector2d pos_new(x, y);
  //     double dist = sdf_map_.getDistance(pos_new);
  //     ROS_INFO("Goal at (%.2f, %.2f), Distance to nearest obstacle: %.2f meters",
  //         x, y, dist);
  //   }
  //   ros::Time end = ros::Time::now();
  //   double dt = (end - start).toSec();
  //   ROS_INFO("Time to compute distance: %.2f ms", dt * 1000);
  // }

  SDFMap2D sdf_map_;
  Eigen::Vector2d size_;
  double resolution_;
  Eigen::Vector2d origin_;
  bool is_map_initialized_;
  ros::NodeHandle node_;
  ros::Subscriber goal_sub_;
  ros::Subscriber obstacle_sub_;

  struct MappingParameters {
    std::string frame_id_;
  } mp_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "sdf_map_2d_node");
  ros::NodeHandle nh("~");

  SDFMap2DNode sdf_map_node(nh);

  ros::spin();

  return 0;
}