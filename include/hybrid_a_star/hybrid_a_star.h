#ifndef HYBRID_A_STAR_HYBRID_A_STAR_H
#define HYBRID_A_STAR_HYBRID_A_STAR_H

#include "util/rs_path.h"
#include "util/state_node.h"
#include "util/dubins_curve.h"
#include "util/timer.h"

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>

// #include <glog/logging.h>

#include <memory>
#include <vector>
#include <map>

class HybridAStar {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    HybridAStar() = delete;

    HybridAStar(double steering_angle, int steering_angle_discrete_num, double segment_length,
                int segment_length_discrete_num, double wheel_base, double steering_penalty,
                double reversing_penalty, double steering_change_penalty, double shot_distance,
                int grid_size_phi = 72);

    ~HybridAStar() = default;

    void Init(double x_lower, double x_upper, double y_lower, double y_upper,
            double state_grid_resolution, double map_grid_resolution = 0.1);

    bool Search(const Vec3d &start_state, const Vec3d &goal_state, int use_dubins_or_rs);

    VectorVec4d GetSearchedTree();

    VectorVec3d GetPath() const;

    __attribute__((unused)) int GetVisitedNodesNumber() const { return visited_node_number_; }

    __attribute__((unused)) double GetPathLength() const;

    __attribute__((unused)) Vec2d CoordinateRounding(const Vec2d &pt) const;

    Vec2i Coordinate2MapGridIndex(const Vec2d &pt) const;

    void SetObstacle(double pt_x, double pt_y);

    void SetObstacle(unsigned int x, unsigned int y);

    void SetVehicleShape(double length, double width, double rear_axle_dist);

    void Reset();

private:
    inline bool HasObstacle(int grid_index_x, int grid_index_y) const;

    inline bool HasObstacle(const Vec2i &grid_index) const;

    bool CheckCollision(const double &x, const double &y, const double &theta);

    inline bool LineCheck(double x0, double y0, double x1, double y1);

    bool AnalyticExpansionsDubins(const std::shared_ptr<StateNode> &current_node_ptr,
                        const std::shared_ptr<StateNode> &goal_node_ptr, double &length);

    bool AnalyticExpansionsRS(const std::shared_ptr<StateNode> &current_node_ptr,
                        const std::shared_ptr<StateNode> &goal_node_ptr, double &length);

    inline double ComputeG(const std::shared_ptr<StateNode> &current_node_ptr,
                        const std::shared_ptr<StateNode> &neighbor_node_ptr) const;

    inline double ComputeH(const std::shared_ptr<StateNode> &current_node_ptr,
                        const std::shared_ptr<StateNode> &terminal_node_ptr);

    inline Vec3i State2Index(const Vec3d &state) const;

    inline Vec2d MapGridIndex2Coordinate(const Vec2i &grid_index) const;

    void GetNeighborNodes(const std::shared_ptr<StateNode> &curr_node_ptr,
                        std::vector<std::shared_ptr<StateNode>> &neighbor_nodes);

    inline void DynamicModel(const double &step_size, const double &phi, double &x, double &y, double &theta) const;

    static inline double Mod2Pi(const double &x);

    bool BeyondBoundary(const Vec2d &pt) const;

    void ReleaseMemory();

private:
    std::vector<uint8_t> map_data_;
    double STATE_GRID_RESOLUTION_{}, MAP_GRID_RESOLUTION_{};
    double ANGULAR_RESOLUTION_{};
    int STATE_GRID_SIZE_X_{}, STATE_GRID_SIZE_Y_{}, STATE_GRID_SIZE_PHI_{};
    int MAP_GRID_SIZE_X_{}, MAP_GRID_SIZE_Y_{};

    double map_x_lower_{}, map_x_upper_{}, map_y_lower_{}, map_y_upper_{};

    std::shared_ptr<StateNode> terminal_node_ptr_;
    std::vector<std::vector<std::vector<std::shared_ptr<StateNode>>>> state_node_map_;

    std::multimap<double, std::shared_ptr<StateNode>> openset_;

    double wheel_base_;
    double segment_length_;
    double move_step_size_;
    double steering_radian_step_size_;
    double steering_radian_;
    double tie_breaker_;

    double turning_radius;

    double shot_distance_;
    int segment_length_discrete_num_;
    int steering_discrete_num_;
    double steering_penalty_;
    double reversing_penalty_;
    double steering_change_penalty_;

    double path_length_ = 0.0;

    std::shared_ptr<DubinsCurveNew> dubins_curve_ptr_;
    std::shared_ptr<RSPath> rs_path_ptr_;

    VecXd vehicle_shape_;
    MatXd vehicle_shape_discrete_;

    double check_collision_use_time = 0.0;
    int num_check_collision = 0;
    int visited_node_number_ = 0;
};

#endif //HYBRID_A_STAR_HYBRID_A_STAR_H