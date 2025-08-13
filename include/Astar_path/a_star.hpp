#ifndef ASTAR_HPP_
#define ASTAR_HPP_

#include "util/type.h"
#include "ros/ros.h"
#include <Eigen/Eigen>
#include <vector>
#include <queue>
#include <memory>

struct GridNode;

class GridMap2D
{
public:
  using Ptr = std::shared_ptr<GridMap2D>;

  GridMap2D(double resolution);
  void initMap(int size_x, int size_y, const Eigen::Vector2d& origin);
  bool setOccupancy(const Eigen::Vector2d& pt, bool occupied);
  bool getOccupancy(const Eigen::Vector2d& pt) const;
  void inflate(double x_inflation, double y_inflation);
  bool isValidIndex(int x, int y) const;
  Eigen::Vector2d indexToCoord(int x, int y) const;
  bool coordToIndex(const Eigen::Vector2d& pt, int& x, int& y) const;
  double getResolution() const { return resolution_; }
  int getSizeX() const { return size_x_; }
  int getSizeY() const { return size_y_; }
  Eigen::Vector2d getOrigin() const { return origin_; }
  void setPath(const std::vector<GridNode*>& path_nodes);
  const std::vector<GridNode*>& getPath() const;
  std::vector<std::vector<bool>> getOccupancy() const{
    return occupancy_;
  }

private:
  double resolution_;
  int size_x_, size_y_;
  Eigen::Vector2d origin_;
  std::vector<std::vector<bool>> occupancy_;
  std::vector<GridNode*> path_nodes_;
};

struct GridNode
{
  using Ptr = GridNode*;
  Eigen::Vector2i index;
  double g_score, f_score;
  Ptr came_from;
  enum State { OPEN, CLOSED, UNVISITED } state;
  int round;

  GridNode() : g_score(std::numeric_limits<double>::infinity()),
               f_score(std::numeric_limits<double>::infinity()),
               came_from(nullptr), state(UNVISITED), round(0) {}
};

class AStarPathPlanner
{
public:
  AStarPathPlanner(double resolution);
  ~AStarPathPlanner() = default;

  void initGridMap(GridMap2D::Ptr map);
  bool aStarSearch(const Eigen::Vector2d& start_pt, const Eigen::Vector2d& end_pt,
                   double timeout_sec = 0.2);
  VectorVec2d getPath();

private:
  double getDiagonalHeuristic(const GridNode* node1, const GridNode* node2);
  bool coordToIndex(const Eigen::Vector2d& pt, Eigen::Vector2i& idx);
  Eigen::Vector2d indexToCoord(const Eigen::Vector2i& idx);
  bool checkOccupancy(const Eigen::Vector2d& pt);
  std::vector<GridNode*> retrievePath(GridNode* current);

  GridMap2D::Ptr grid_map_;
  Eigen::Vector2i pool_size_;
  std::vector<std::vector<std::unique_ptr<GridNode>>> grid_node_map_;
  double resolution_, step_size_, robot_width_;
  int rounds_;
  std::priority_queue<GridNode*, std::vector<GridNode*>,
                      std::function<bool(GridNode*, GridNode*)>> open_set_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif // ASTAR_HPP_