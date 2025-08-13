#include "Astar_path/a_star_node.h"

#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

AStarNode::AStarNode(ros::NodeHandle &nh) {

    double w_ref = nh.param("planner/w_ref", 0.0);

    grid_map_ptr_ = std::make_shared<GridMap2D>(0.2);
    astar_planner_ptr_ = std::make_shared<AStarPathPlanner>(0.2);

    costmap_sub_ptr_ = std::make_shared<CostMapSubscriber>(nh, "/map", 1);
    init_pose_sub_ptr_ = std::make_shared<InitPoseSubscriber2D>(nh, "/initialpose", 1);
    goal_pose_sub_ptr_ = std::make_shared<GoalPoseSubscriber2D>(nh, "/move_base_simple/goal", 1);

    sdf_map_ptr_ = std::make_shared<SDFMap2D>();
    sdf_map_refer_ptr_ = std::make_shared<SDFMap2D>();
    smoother_ptr_ = std::make_shared<Smoother>();
    smoother_ptr_->set_w_ref(w_ref);

    path_pub_ = nh.advertise<nav_msgs::Path>("searched_path", 1);
    smooth_path_pub_ = nh.advertise<nav_msgs::Path>("smooth_path", 1);
    searched_tree_pub_ = nh.advertise<visualization_msgs::Marker>("searched_tree", 1);
    vehicle_path_pub_ = nh.advertise<visualization_msgs::MarkerArray>("vehicle_path", 1);
    inflated_occupancy_pub_ = nh.advertise<sensor_msgs::PointCloud2>("inflated_occupancy", 1);

    obstacle_points_sub_ = nh.subscribe("/obstacle_points", 10, &AStarNode::ObstaclePointsCallback, this);
    has_map_ = false;
    has_inflate_ = false;
    has_sdf_map_ = false;
}

void AStarNode::ObstaclePointsCallback(const sensor_msgs::PointCloud2ConstPtr& point_cloud_msg) {
    obstacle_points_deque_.push_back(point_cloud_msg);
}

void AStarNode::Run() {
    ReadData();

    if (!has_map_) {
        if (costmap_deque_.empty()) {
            return;
        }

        current_costmap_ptr_ = costmap_deque_.front();
        costmap_deque_.pop_front();

        const double map_resolution = static_cast<float>(current_costmap_ptr_->info.resolution);

        double x_lower = current_costmap_ptr_->info.origin.position.x;
        double x_upper = x_lower + current_costmap_ptr_->info.width * map_resolution;
        double y_lower = current_costmap_ptr_->info.origin.position.y;
        double y_upper = y_lower + current_costmap_ptr_->info.height * map_resolution;

        x_offset_ = x_lower;
        y_offset_ = y_lower;
        std::cout << "x_lower: " << x_lower << " x_upper: " << x_upper << " y_lower: " << y_lower << " y_upper: " << y_upper << std::endl;

        grid_map_ptr_->initMap(250, 250, Eigen::Vector2d(0.0, 0.0));


        unsigned int map_w = std::floor(current_costmap_ptr_->info.width);
        unsigned int map_h = std::floor(current_costmap_ptr_->info.height);
        for (unsigned int w = 0; w < map_w; ++w) {
            for (unsigned int h = 0; h < map_h; ++h) {
                if (current_costmap_ptr_->data[h * current_costmap_ptr_->info.width + w]) {
                    grid_map_ptr_->setOccupancy(Eigen::Vector2d(w, h), true);
                }
            }
        }
        astar_planner_ptr_->initGridMap(grid_map_ptr_);
        has_map_ = true;
    }
    // 处理障碍物点云
    while (!obstacle_points_deque_.empty()) {
        auto point_cloud_msg = obstacle_points_deque_.front();
        obstacle_points_deque_.pop_front();

        // 转换为 PCL 点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*point_cloud_msg, *cloud);

        // 将每个点设置为障碍物
        for (const auto& point : cloud->points) {
            grid_map_ptr_->setOccupancy(Eigen::Vector2d(point.x, point.y), true);
            obstacles.emplace_back(point.x, point.y);
        }
        // 障碍物点云膨胀
        if(!has_inflate_){
            ros::Time start = ros::Time::now();
            grid_map_ptr_->inflate(0.5, 0.5);
            ros::Time end = ros::Time::now();
            std::cout << "map inflated complete!" << std::endl;
            std::cout << "map inflated time: " << (end - start).toSec() * 1000 << "ms" << std::endl;
            has_inflate_ = true;
        }
    }
    for(double i = 5.0; i < 45.0; i += 0.5){
        refer_obstacles.emplace_back(i, 40.0);
    }
    
    if(!has_sdf_map_){
        ros::Time start = ros::Time::now();
        sdf_map_ptr_->initMap(Eigen::Vector2d(0.0, 0.0), Eigen::Vector2i(250, 250), 0.2, obstacles); 
        sdf_map_refer_ptr_->initMap(Eigen::Vector2d(0.0, 0.0), Eigen::Vector2i(250, 250), 0.2, refer_obstacles);
        ros::Time end = ros::Time::now();
        std::cout << "\033[1;32mTime to initialize map : " << (end - start).toSec() * 1000 << "ms\033[0m" << std::endl;
        has_sdf_map_ = true;
    }

    costmap_deque_.clear();

    while (HasStartPose() && HasGoalPose()) {
        InitPoseData();

        double start_yaw = tf::getYaw(current_init_pose_ptr_->pose.pose.orientation);
        double goal_yaw = tf::getYaw(current_goal_pose_ptr_->pose.orientation);

        Vec3d start_state = Vec3d(
                current_init_pose_ptr_->pose.pose.position.x - x_offset_,
                current_init_pose_ptr_->pose.pose.position.y - y_offset_,
                start_yaw
        );
        Vec3d goal_state = Vec3d(
                current_goal_pose_ptr_->pose.position.x - x_offset_,
                current_goal_pose_ptr_->pose.position.y - y_offset_,
                goal_yaw
        );
        std::cout << "start_state: " << start_state.transpose() << std::endl;
        std::cout << "goal_state: " << goal_state.transpose() << std::endl;
        ros::Time Astar_start = ros::Time::now();
        if (astar_planner_ptr_->aStarSearch(start_state.head(2), goal_state.head(2))) {
            auto Astar_path = astar_planner_ptr_->getPath();
            ros::Time Astar_end = ros::Time::now();
            std::cout << "\033[1;32mAstar time: " << (Astar_end - Astar_start).toSec() * 1000 << "ms\033[0m" << std::endl;

            VectorVec3d path = convertPathTo3DWithYaw(Astar_path);
            PublishInflatedOccupancy();
            PublishPath(path);

            // 对路径点下采样
            VectorVec3d sample_path;
            for(int i = 0; i < static_cast<int>(path.size()) - 5; i += 5){
                sample_path.push_back(path[i]);
            }
            // 添加终点
            sample_path.push_back(goal_state);

            ros::Time optimize_start = ros::Time::now();
            smoother_ptr_->optimize(sdf_map_ptr_, sdf_map_refer_ptr_, sample_path);
            VectorVec3d smooth_path;
            smoother_ptr_->getSmoothPath(smooth_path);

            VectorVec3d quintic_path = QuinticSpline::fitQuinticSpline(smooth_path, 0.1, 1.0);

            ros::Time optimize_end = ros::Time::now();
            double optimize_time = (optimize_end - optimize_start).toSec();
            std::cout << "\033[1;32mTime to optimize path : " << optimize_time * 1000 << "ms\033[0m" << std::endl;
            PublishSmoothPath(quintic_path);

            PublishVehiclePath(quintic_path, 2.0, 1.0, 3u);

            nav_msgs::Path path_ros;
            geometry_msgs::PoseStamped pose_stamped;

            for (const auto &pose: quintic_path) {
                pose_stamped.header.frame_id = "world";
                pose_stamped.pose.position.x = pose.x();
                pose_stamped.pose.position.y = pose.y();
                pose_stamped.pose.position.z = 0.0;

                pose_stamped.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, pose.z());

                path_ros.poses.emplace_back(pose_stamped);
            }

            path_ros.header.frame_id = "world";
            path_ros.header.stamp = ros::Time::now();
            static tf::TransformBroadcaster transform_broadcaster;
            for (const auto &pose: path_ros.poses) {
                tf::Transform transform;
                transform.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y, 0.0));

                tf::Quaternion q;
                q.setX(pose.pose.orientation.x);
                q.setY(pose.pose.orientation.y);
                q.setZ(pose.pose.orientation.z);
                q.setW(pose.pose.orientation.w);
                transform.setRotation(q);

                transform_broadcaster.sendTransform(tf::StampedTransform(transform,
                                                                         ros::Time::now(), "world",
                                                                         "ground_link")
                );

                ros::Duration(0.03).sleep();
            }
        }
        else{
            // 打印红色提示
            std::cout << "\033[31m No path found! \033[0m" << std::endl;
        }

        // debug
        // std::cout << "visited nodes: " << astar_planner_ptr_->GetVisitedNodesNumber() << std::endl;
        // astar_planner_ptr_->Reset();
    }
}

VectorVec3d AStarNode::convertPathTo3DWithYaw(const VectorVec2d& path_2d) {
    VectorVec3d path_3d;
    if (path_2d.size() < 2) {
        // 如果路径点少于2个，无法计算偏航角，直接返回空路径
        return path_3d;
    }

    path_3d.reserve(path_2d.size()); // 预分配空间

    for (size_t i = 0; i < path_2d.size(); ++i) {
        double yaw = 0.0;
        Vec2d current = path_2d[i];
        Vec3d point_3d(current.x(), current.y(), 0.0);

        if (i == 0) {
            // 起点：使用后一个点计算偏航角
            Vec2d next = path_2d[i + 1];
            yaw = std::atan2(next.y() - current.y(), next.x() - current.x());
        } else if (i == path_2d.size() - 1) {
            // 终点：使用前一个点计算偏航角
            Vec2d prev = path_2d[i - 1];
            yaw = std::atan2(current.y() - prev.y(), current.x() - prev.x());
        } else {
            // 中间点：使用前后点连线方向计算偏航角
            Vec2d prev = path_2d[i - 1];
            Vec2d next = path_2d[i + 1];
            yaw = std::atan2(next.y() - prev.y(), next.x() - prev.x());
        }

        point_3d.z() = yaw; // 设置偏航角
        path_3d.push_back(point_3d);
    }

    return path_3d;
}

void AStarNode::ReadData() {
    costmap_sub_ptr_->ParseData(costmap_deque_);
    init_pose_sub_ptr_->ParseData(init_pose_deque_);
    goal_pose_sub_ptr_->ParseData(goal_pose_deque_);
}

void AStarNode::InitPoseData() {
    current_init_pose_ptr_ = init_pose_deque_.front();
    init_pose_deque_.pop_front();

    current_goal_pose_ptr_ = goal_pose_deque_.front();
    goal_pose_deque_.pop_front();
}

bool AStarNode::HasGoalPose() {
    return !goal_pose_deque_.empty();
}

bool AStarNode::HasStartPose() {
    return !init_pose_deque_.empty();
}

void AStarNode::PublishPath(const VectorVec3d &path) {
    nav_msgs::Path nav_path;

    geometry_msgs::PoseStamped pose_stamped;
    for (const auto &pose: path) {
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = pose.x();
        pose_stamped.pose.position.y = pose.y();
        pose_stamped.pose.position.z = 0.0;
        pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(pose.z());
        // pose_stamped.pose.orientation.w = 1.0;

        nav_path.poses.emplace_back(pose_stamped);
    }

    nav_path.header.frame_id = "world";
    nav_path.header.stamp = timestamp_;

    path_pub_.publish(nav_path);
}

void AStarNode::PublishSmoothPath(const VectorVec3d &path) {
    nav_msgs::Path nav_path;

    geometry_msgs::PoseStamped pose_stamped;
    for (const auto &pose: path) {
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = pose.x();
        pose_stamped.pose.position.y = pose.y();
        pose_stamped.pose.position.z = 0.0;
        pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(pose.z());

        nav_path.poses.emplace_back(pose_stamped);
    }

    nav_path.header.frame_id = "world";
    nav_path.header.stamp = timestamp_;

    smooth_path_pub_.publish(nav_path);
}

void AStarNode::PublishVehiclePath(const VectorVec3d &path, double width,
                                   double length, unsigned int vehicle_interval = 3u) {
    visualization_msgs::MarkerArray vehicle_array;

    for (unsigned int i = 0; i < path.size(); i += vehicle_interval) {
    visualization_msgs::Marker vehicle;

    if (i == 0) {
    vehicle.action = 3;
    }

    vehicle.header.frame_id = "world";
    vehicle.header.stamp = ros::Time::now();
    vehicle.type = visualization_msgs::Marker::CUBE;
    vehicle.id = static_cast<int>(i / vehicle_interval);
    vehicle.scale.x = width;
    vehicle.scale.y = length;
    vehicle.scale.z = 0.01;
    vehicle.color.a = 0.1;

    vehicle.color.r = 1.0;
    vehicle.color.b = 0.0;
    vehicle.color.g = 0.0;

    vehicle.pose.position.x = path[i].x();
    vehicle.pose.position.y = path[i].y();
    vehicle.pose.position.z = 0.0;

    vehicle.pose.orientation = tf::createQuaternionMsgFromYaw(path[i].z());
    vehicle_array.markers.emplace_back(vehicle);
}

vehicle_path_pub_.publish(vehicle_array);
}

// 发布膨胀点云
void AStarNode::PublishInflatedOccupancy() {
    sensor_msgs::PointCloud2 point_cloud_msg;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    auto getOccupancy = grid_map_ptr_->getOccupancy();

    for (int x = 0; x < grid_map_ptr_->getSizeX(); ++x) {
        for (int y = 0; y < grid_map_ptr_->getSizeY(); ++y) {
            if (getOccupancy[x][y]) {
                Eigen::Vector2d coord = grid_map_ptr_->indexToCoord(x, y);
                pcl::PointXYZ point;
                point.x = coord.x() + x_offset_;
                point.y = coord.y() + y_offset_;
                point.z = 0.0;
                cloud.points.push_back(point);
            }
        }
    }

    pcl::toROSMsg(cloud, point_cloud_msg);
    point_cloud_msg.header.frame_id = "world";
    point_cloud_msg.header.stamp = ros::Time::now();

    inflated_occupancy_pub_.publish(point_cloud_msg);
}
