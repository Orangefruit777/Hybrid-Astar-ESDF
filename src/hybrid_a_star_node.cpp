#include "hybrid_a_star/hybrid_a_star_node.h"

#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

__attribute__((unused)) double Mod2Pi(const double &x) {
    double v = fmod(x, 2 * M_PI);

    if (v < -M_PI) {
        v += 2.0 * M_PI;
    } else if (v > M_PI) {
        v -= 2.0 * M_PI;
    }

    return v;
}

HybridAStarFlow::HybridAStarFlow(ros::NodeHandle &nh) {
    double steering_angle = nh.param("planner/steering_angle", 10);
    int steering_angle_discrete_num = nh.param("planner/steering_angle_discrete_num", 1);
    double wheel_base = nh.param("planner/wheel_base", 1.0);
    double segment_length = nh.param("planner/segment_length", 1.6);
    int segment_length_discrete_num = nh.param("planner/segment_length_discrete_num", 8);
    double steering_penalty = nh.param("planner/steering_penalty", 1.05);
    double steering_change_penalty = nh.param("planner/steering_change_penalty", 1.5);
    double reversing_penalty = nh.param("planner/reversing_penalty", 2.0);
    double shot_distance = nh.param("planner/shot_distance", 5.0);
    double w_ref = nh.param("planner/w_ref", 0.0);
    use_dubins_or_rs_ = nh.param("planner/use_dubins_or_rs", 0);

    kinodynamic_astar_searcher_ptr_ = std::make_shared<HybridAStar>(
            steering_angle, steering_angle_discrete_num, segment_length, segment_length_discrete_num, wheel_base,
            steering_penalty, reversing_penalty, steering_change_penalty, shot_distance
    );
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


    obstacle_points_sub_ = nh.subscribe("/obstacle_points", 10, &HybridAStarFlow::ObstaclePointsCallback, this);
    has_map_ = false;
    has_sdf_map_ = false;
}

void HybridAStarFlow::ObstaclePointsCallback(const sensor_msgs::PointCloud2ConstPtr& point_cloud_msg) {
    obstacle_points_deque_.push_back(point_cloud_msg);
}

void HybridAStarFlow::Run() {
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

        kinodynamic_astar_searcher_ptr_->Init(0, x_upper - x_lower, 0, y_upper - y_lower, 1.0, map_resolution);

        // kinodynamic_astar_searcher_ptr_->Init(
        //         current_costmap_ptr_->info.origin.position.x,
        //         current_costmap_ptr_->info.origin.position.x + 1.0 * current_costmap_ptr_->info.width * current_costmap_ptr_->info.resolution,
        //         current_costmap_ptr_->info.origin.position.y,
        //         current_costmap_ptr_->info.origin.position.y + 1.0 * current_costmap_ptr_->info.height * current_costmap_ptr_->info.resolution,
        //         1.0, map_resolution
        // );


        unsigned int map_w = std::floor(current_costmap_ptr_->info.width);
        unsigned int map_h = std::floor(current_costmap_ptr_->info.height);
        for (unsigned int w = 0; w < map_w; ++w) {
            for (unsigned int h = 0; h < map_h; ++h) {
                if (current_costmap_ptr_->data[h * current_costmap_ptr_->info.width + w]) {
                    kinodynamic_astar_searcher_ptr_->SetObstacle(w, h);
                }
            }
        }
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
            kinodynamic_astar_searcher_ptr_->SetObstacle(point.x, point.y);
            obstacles.emplace_back(point.x, point.y);
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
        std::cout << "\033[1;32mESDF map generation time: " << (end - start).toSec() * 1000 << " ms\033[0m" << std::endl;
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
        ros::Time hybrid_start = ros::Time::now();
        if (kinodynamic_astar_searcher_ptr_->Search(start_state, goal_state, use_dubins_or_rs_)) {
            auto path = kinodynamic_astar_searcher_ptr_->GetPath();
            ros::Time hybrid_end = ros::Time::now();
            std::cout << "\033[1;32mhybrid astar time : " << (hybrid_end - hybrid_start).toSec() * 1000 << " ms\033[0m" << std::endl;
            
            PublishPath(path);
            PublishSearchedTree(kinodynamic_astar_searcher_ptr_->GetSearchedTree());

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
            ros::Time optimize_end = ros::Time::now();
            std::cout << "\033[1;32moptimize time : " << (optimize_end - optimize_start).toSec() * 1000 << " ms\033[0m" << std::endl;

            ros::Time quintic_sample_start = ros::Time::now();
            VectorVec3d quintic_path = QuinticSpline::fitQuinticSpline(smooth_path, 0.1, 1.0);
            ros::Time quintic_sample_end = ros::Time::now();
            std::cout << "quintic sample time : " << (quintic_sample_end - quintic_sample_start).toSec() * 1000 << " ms" << std::endl;

            std::cout << "-----------------------------" << std::endl;

            PublishSmoothPath(quintic_path);

            // 修改车辆显示方框， path or quintic_path
            // PublishVehiclePath(path, 2.0, 1.0, 1u);
            PublishVehiclePath(quintic_path, 2.0, 1.0, 3u);

            nav_msgs::Path path_ros;
            geometry_msgs::PoseStamped pose_stamped;

            // 修改车辆行驶路径， path or quintic_path
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

                ros::Duration(0.02).sleep();
                // ros::Duration(0.10).sleep();
            }
        }
        else{
            // 打印红色提示
            std::cout << "\033[31m No path found! \033[0m" << std::endl;
        }

        // debug
//        std::cout << "visited nodes: " << kinodynamic_astar_searcher_ptr_->GetVisitedNodesNumber() << std::endl;
        kinodynamic_astar_searcher_ptr_->Reset();
    }
}

// unused 对平滑路径进行精细采样，恢复到原始路径的点数，修改后不再使用
VectorVec3d HybridAStarFlow::RefinePath(const VectorVec3d& smooth_path, int original_size) {
    VectorVec3d refined_path;
    int smooth_size = smooth_path.size();

    // 计算每段需要生成的点数（包括端点）
    double points_per_segment = static_cast<double>(original_size - 1) / (smooth_size - 1);

    // 添加第一个点（yaw 稍后计算）
    refined_path.push_back(smooth_path[0]);

    // 线性插值生成 x 和 y 坐标
    for (int i = 0; i < smooth_size - 1; ++i) {
        const Vec3d& p1 = smooth_path[i];
        const Vec3d& p2 = smooth_path[i + 1];

        // 计算当前段的点数（包括起点，不包括终点）
        int num_points = std::round(points_per_segment);
        if (i == smooth_size - 2) {
            // 最后一个段，调整点数以确保总点数精确匹配
            num_points = original_size - refined_path.size() - 1;
        }

        // 线性插值生成中间点的 x 和 y
        for (int j = 1; j < num_points; ++j) {
            double t = static_cast<double>(j) / num_points;
            Vec3d interpolated_point;
            interpolated_point.x() = (1 - t) * p1.x() + t * p2.x();
            interpolated_point.y() = (1 - t) * p1.y() + t * p2.y();
            interpolated_point.z() = 0.0; // yaw 暂时设为 0，稍后计算
            refined_path.push_back(interpolated_point);
        }
        // 添加当前段的终点
        refined_path.push_back(p2);
    }

    // 计算所有点的 yaw，基于前后两点的连线向量
    for (size_t i = 0; i < refined_path.size(); ++i) {
        if (i == 0) {
            refined_path[i].z() = smooth_path[0].z();
        } else if (i == refined_path.size() - 1) {
            refined_path[i].z() = smooth_path[smooth_size - 1].z();
        } else {
            // 中间点：使用 refined_path[i+1] - refined_path[i-1]
            double delta_x = refined_path[i + 1].x() - refined_path[i - 1].x();
            double delta_y = refined_path[i + 1].y() - refined_path[i - 1].y();
            if (std::abs(delta_x) < 1e-6 && std::abs(delta_y) < 1e-6) {
                // 如果前后点重合，继承前一点的 yaw
                refined_path[i].z() = refined_path[i - 1].z();
            } else {
                refined_path[i].z() = std::atan2(delta_y, delta_x);
            }
        }
    }

    return refined_path;
}

void HybridAStarFlow::ReadData() {
    costmap_sub_ptr_->ParseData(costmap_deque_);
    init_pose_sub_ptr_->ParseData(init_pose_deque_);
    goal_pose_sub_ptr_->ParseData(goal_pose_deque_);
}

void HybridAStarFlow::InitPoseData() {
    current_init_pose_ptr_ = init_pose_deque_.front();
    init_pose_deque_.pop_front();

    current_goal_pose_ptr_ = goal_pose_deque_.front();
    goal_pose_deque_.pop_front();
}

bool HybridAStarFlow::HasGoalPose() {
    return !goal_pose_deque_.empty();
}

bool HybridAStarFlow::HasStartPose() {
    return !init_pose_deque_.empty();
}

void HybridAStarFlow::PublishPath(const VectorVec3d &path) {
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

    path_pub_.publish(nav_path);
}

void HybridAStarFlow::PublishSmoothPath(const VectorVec3d &path) {
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

void HybridAStarFlow::PublishVehiclePath(const VectorVec3d &path, double width,
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

void HybridAStarFlow::PublishSearchedTree(const VectorVec4d &searched_tree) {
    visualization_msgs::Marker tree_list;
    tree_list.header.frame_id = "world";
    tree_list.header.stamp = ros::Time::now();
    tree_list.type = visualization_msgs::Marker::LINE_LIST;
    tree_list.action = visualization_msgs::Marker::ADD;
    tree_list.ns = "searched_tree";
    tree_list.scale.x = 0.02;

    tree_list.color.a = 1.0;
    tree_list.color.r = 0;
    tree_list.color.g = 0;
    tree_list.color.b = 0;

    tree_list.pose.orientation.w = 1.0;
    tree_list.pose.orientation.x = 0.0;
    tree_list.pose.orientation.y = 0.0;
    tree_list.pose.orientation.z = 0.0;

    geometry_msgs::Point point;
    for (const auto &i: searched_tree) {
        point.x = i.x();
        point.y = i.y();
        point.z = 0.0;
        tree_list.points.emplace_back(point);

        point.x = i.z();
        point.y = i.w();
        point.z = 0.0;
        tree_list.points.emplace_back(point);
    }

    searched_tree_pub_.publish(tree_list);
}