#include "Astar_path/a_star.hpp"

#include <iostream>
#include <cmath>
#include <algorithm>
#include <iterator>
#include <limits>
#include <chrono>

GridMap2D::GridMap2D(double resolution) : resolution_(resolution), size_x_(0), size_y_(0)
{
}

void GridMap2D::initMap(int size_x, int size_y, const Eigen::Vector2d& origin)
{
  size_x_ = size_x;
  size_y_ = size_y;
  origin_ = origin;
  occupancy_.resize(size_x_, std::vector<bool>(size_y_, false));
  path_nodes_.clear();
}

bool GridMap2D::setOccupancy(const Eigen::Vector2d& pt, bool occupied)
{
  int x, y;
  if (!coordToIndex(pt, x, y)) return false;
  occupancy_[x][y] = occupied;
  return true;
}

bool GridMap2D::getOccupancy(const Eigen::Vector2d& pt) const
{
  int x, y;
  if (!coordToIndex(pt, x, y)) return true;
  return occupancy_[x][y];
}

void GridMap2D::inflate(double x_inflation, double y_inflation)
{
  std::vector<std::vector<bool>> new_occupancy = occupancy_;
  int x_cells = std::ceil(x_inflation / resolution_);
  int y_cells = std::ceil(y_inflation / resolution_);

  std::cout << "x_cells: " << x_cells << " y_cells: " << y_cells << std::endl;

  for (int x = 0; x < size_x_; ++x) {
    for (int y = 0; y < size_y_; ++y) {
      if (occupancy_[x][y]) {
        // 获取当前栅格的中心坐标
        Eigen::Vector2d center = indexToCoord(x, y);
        // 检查以当前栅格为中心，沿 x 和 y 方向的矩形区域
        for (int dx = -x_cells; dx <= x_cells; ++dx) {
          for (int dy = -y_cells; dy <= y_cells; ++dy) {
            // 计算候选点的坐标：沿 x 和 y 方向的偏移
            Eigen::Vector2d offset(dx * resolution_, dy * resolution_);
            Eigen::Vector2d candidate = center + offset + Eigen::Vector2d(0.001, 0.001);
            int nx, ny;
            if (coordToIndex(candidate, nx, ny)) {
                new_occupancy[nx][ny] = true;
            }
          }
        }
      }
    }
  }
  occupancy_ = new_occupancy;
}

bool GridMap2D::isValidIndex(int x, int y) const
{
  return x >= 0 && x < size_x_ && y >= 0 && y < size_y_;
}

Eigen::Vector2d GridMap2D::indexToCoord(int x, int y) const
{
  return origin_ + Eigen::Vector2d(x * resolution_, y * resolution_);
}

bool GridMap2D::coordToIndex(const Eigen::Vector2d& pt, int& x, int& y) const
{
  x = std::floor((pt.x() - origin_.x()) / resolution_);
  y = std::floor((pt.y() - origin_.y()) / resolution_);
  return isValidIndex(x, y);
}

void GridMap2D::setPath(const std::vector<GridNode*>& path_nodes)
{
  path_nodes_ = path_nodes;
}

const std::vector<GridNode*>& GridMap2D::getPath() const
{
  return path_nodes_;
}

AStarPathPlanner::AStarPathPlanner(double resolution)
    : resolution_(resolution), rounds_(0),
      open_set_([](GridNode* a, GridNode* b) { return a->f_score > b->f_score; })
{
}

void AStarPathPlanner::initGridMap(GridMap2D::Ptr map)
{
  grid_map_ = map;
  pool_size_ = Eigen::Vector2i(map->getSizeX(), map->getSizeY());

  grid_node_map_.clear();
  grid_node_map_.resize(pool_size_.x());
  for (int x = 0; x < pool_size_.x(); ++x) {
    grid_node_map_[x].resize(pool_size_.y());
    for (int y = 0; y < pool_size_.y(); ++y) {
      grid_node_map_[x][y] = std::make_unique<GridNode>();
      grid_node_map_[x][y]->index = Eigen::Vector2i(x, y);
    }
  }
}

double AStarPathPlanner::getDiagonalHeuristic(const GridNode* node1, const GridNode* node2)
{
  double dx = std::abs(node1->index.x() - node2->index.x());
  double dy = std::abs(node1->index.y() - node2->index.y());
  double diag = std::min(dx, dy);
  return (diag * std::sqrt(2.0) + std::abs(dx - dy)) * resolution_;
}

bool AStarPathPlanner::coordToIndex(const Eigen::Vector2d& pt, Eigen::Vector2i& idx)
{
  int x, y;
  if (!grid_map_->coordToIndex(pt, x, y)) return false;
  idx = Eigen::Vector2i(x, y);
  return true;
}

Eigen::Vector2d AStarPathPlanner::indexToCoord(const Eigen::Vector2i& idx)
{
  return grid_map_->indexToCoord(idx.x(), idx.y());
}

bool AStarPathPlanner::checkOccupancy(const Eigen::Vector2d& pt)
{
  return grid_map_->getOccupancy(pt);
}

std::vector<GridNode*> AStarPathPlanner::retrievePath(GridNode* current)
{
  std::vector<GridNode*> path;
  while (current != nullptr) {
    path.push_back(current);
    current = current->came_from;
  }
  return path;
}

bool AStarPathPlanner::aStarSearch(const Eigen::Vector2d& start_pt, const Eigen::Vector2d& end_pt,
                                   double timeout_sec)
{
  auto time_start = std::chrono::steady_clock::now();
  ++rounds_;

  Eigen::Vector2i start_idx, end_idx;
  if (!coordToIndex(start_pt, start_idx) || !coordToIndex(end_pt, end_idx)) {
    std::cout <<"A* failed: invalid start or end point"<<std::endl;
    return false;
  }

  Eigen::Vector2d adjusted_start = start_pt, adjusted_end = end_pt;
  if (checkOccupancy(adjusted_start)) {
    do {
      adjusted_start += (end_pt - adjusted_start).normalized() * step_size_;
      if (!coordToIndex(adjusted_start, start_idx)) return false;
    } while (checkOccupancy(adjusted_start));
  }
  if (checkOccupancy(adjusted_end)) {
    do {
      adjusted_end += (start_pt - adjusted_end).normalized() * step_size_;
      if (!coordToIndex(adjusted_end, end_idx)) return false;
    } while (checkOccupancy(adjusted_end));
  }

  for (int x = 0; x < pool_size_.x(); ++x) {
    for (int y = 0; y < pool_size_.y(); ++y) {
      grid_node_map_[x][y]->g_score = std::numeric_limits<double>::infinity();
      grid_node_map_[x][y]->f_score = std::numeric_limits<double>::infinity();
      grid_node_map_[x][y]->came_from = nullptr;
      grid_node_map_[x][y]->state = GridNode::UNVISITED;
      grid_node_map_[x][y]->round = rounds_;
    }
  }

  GridNode* start_node = grid_node_map_[start_idx.x()][start_idx.y()].get();
  GridNode* end_node = grid_node_map_[end_idx.x()][end_idx.y()].get();
  start_node->g_score = 0.0;
  start_node->f_score = getDiagonalHeuristic(start_node, end_node);
  start_node->state = GridNode::OPEN;
  start_node->round = rounds_;
  while (!open_set_.empty()) open_set_.pop();
  open_set_.push(start_node);

  while (!open_set_.empty()) {
    GridNode* current = open_set_.top();
    open_set_.pop();
    if (current->state == GridNode::CLOSED) continue;
    current->state = GridNode::CLOSED;

    if (current->index == end_node->index) {
      grid_map_->setPath(retrievePath(current));
      return true;
    }

    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        if (dx == 0 && dy == 0) continue;
        Eigen::Vector2i neighbor_idx = current->index + Eigen::Vector2i(dx, dy);
        if (!grid_map_->isValidIndex(neighbor_idx.x(), neighbor_idx.y())) continue;

        GridNode* neighbor = grid_node_map_[neighbor_idx.x()][neighbor_idx.y()].get();
        if (neighbor->state == GridNode::CLOSED) continue;

        if (checkOccupancy(indexToCoord(neighbor_idx))) continue;

        double cost = std::sqrt(dx * dx + dy * dy) * resolution_;
        double tentative_g_score = current->g_score + cost;

        if (tentative_g_score < neighbor->g_score) {
          neighbor->came_from = current;
          neighbor->g_score = tentative_g_score;
          neighbor->f_score = tentative_g_score + getDiagonalHeuristic(neighbor, end_node);
          neighbor->state = GridNode::OPEN;
          open_set_.push(neighbor);
        }
      }
    }

    if (std::chrono::duration<double>(std::chrono::steady_clock::now() - time_start).count() > timeout_sec) {
        std::cout << "A* search timed out after " << timeout_sec << " seconds" << std::endl;
        return false;
    }
  }

  std::cout<<"A* failed: no path found"<<std::endl;
  return false;
}

VectorVec2d AStarPathPlanner::getPath()
{
    VectorVec2d path;
    for (const auto* node : grid_map_->getPath())
    {
        path.push_back(indexToCoord(node->index));
    }
    std::reverse(path.begin(), path.end());

    return path;
}
