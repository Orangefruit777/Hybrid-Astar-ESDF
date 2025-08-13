#ifndef A_STAR_FLOW_H
#define A_STAR_FLOW_H

#include "a_star.hpp"
#include "util/costmap_subscriber.h"
#include "util/init_pose_subscriber.h"
#include "util/goal_pose_subscriber.h"


#include <sensor_msgs/PointCloud2.h> // 使用 PointCloud2
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "plan_env/sdf_map_2d.h"
#include "hybrid_a_star/quintic_spline.h"
#include "hybrid_a_star/smooth.h"

#include <ros/ros.h>

class AStarNode {
public:
    AStarNode() = default;

    explicit AStarNode(ros::NodeHandle &nh);

    void Run();

private:
    void InitPoseData();

    void ReadData();

    bool HasStartPose();

    bool HasGoalPose();

    // 新增点云回调函数
    void ObstaclePointsCallback(const sensor_msgs::PointCloud2ConstPtr& point_cloud_msg);

    void PublishPath(const VectorVec3d &path);

    void PublishSmoothPath(const VectorVec3d &path);

    void PublishVehiclePath(const VectorVec3d &path, double width,
                            double length, unsigned int vehicle_interval);

    VectorVec3d convertPathTo3DWithYaw(const VectorVec2d& path_2d);

    void PublishInflatedOccupancy();

private:
    std::shared_ptr<GridMap2D> grid_map_ptr_;
    std::shared_ptr<AStarPathPlanner> astar_planner_ptr_;
    std::shared_ptr<CostMapSubscriber> costmap_sub_ptr_;
    std::shared_ptr<InitPoseSubscriber2D> init_pose_sub_ptr_;
    std::shared_ptr<GoalPoseSubscriber2D> goal_pose_sub_ptr_;
    std::shared_ptr<SDFMap2D> sdf_map_ptr_;
    std::shared_ptr<SDFMap2D> sdf_map_refer_ptr_;
    std::shared_ptr<Smoother> smoother_ptr_;

    ros::Publisher path_pub_;
    ros::Publisher smooth_path_pub_;
    ros::Publisher searched_tree_pub_;
    ros::Publisher vehicle_path_pub_;
    ros::Publisher inflated_occupancy_pub_;

    // 新增点云订阅器
    ros::Subscriber obstacle_points_sub_;
    std::deque<sensor_msgs::PointCloud2ConstPtr> obstacle_points_deque_;

    std::deque<geometry_msgs::PoseWithCovarianceStampedPtr> init_pose_deque_;
    std::deque<geometry_msgs::PoseStampedPtr> goal_pose_deque_;
    std::deque<nav_msgs::OccupancyGridPtr> costmap_deque_;

    geometry_msgs::PoseWithCovarianceStampedPtr current_init_pose_ptr_;
    geometry_msgs::PoseStampedPtr current_goal_pose_ptr_;
    nav_msgs::OccupancyGridPtr current_costmap_ptr_;

    ros::Time timestamp_;

    bool has_map_{};
    bool has_inflate_;
    bool has_sdf_map_;
    std::vector<Eigen::Vector2d> obstacles;
    std::vector<Eigen::Vector2d> refer_obstacles;

    double x_offset_, y_offset_;
};

#endif // A_STAR_FLOW_H
