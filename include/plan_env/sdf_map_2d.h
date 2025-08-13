#ifndef _SDF_MAP_2D_H
#define _SDF_MAP_2D_H

#include <Eigen/Eigen>
#include <iostream>
#include <vector>

class SDFMap2D {
public:
  SDFMap2D() {}
  ~SDFMap2D() {}

  // Initialize the 2D map with size, resolution, and obstacle points
  void initMap(const Eigen::Vector2d& origin,  const Eigen::Vector2i& size, double resolution,
               const std::vector<Eigen::Vector2d>& obstacles);

  // Update 2D ESDF based on occupancy buffer
  void updateESDF2d();

  // Get distance at a given position
  double getDistance(const Eigen::Vector2d& pos);
  
  // 以下四个函数用于获取梯度信息
  void getSurroundPts(const Eigen::Vector2d& pos, Eigen::Vector2d pts[2][2], Eigen::Vector2d& diff);

  void interpolateBilinear(double values[2][2], const Eigen::Vector2d& diff, double& value, Eigen::Vector2d& grad);

  void getSurroundDistance(Eigen::Vector2d pts[2][2], double dists[2][2]);
  
  // Get gradient at a given position
  void evaluateEDTWithGrad(const Eigen::Vector2d& pos, double& dist, Eigen::Vector2d& grad);


private:
  struct MappingParameters {
    Eigen::Vector2d map_origin_, map_size_;
    Eigen::Vector2d map_min_boundary_, map_max_boundary_;
    Eigen::Vector2i map_voxel_num_;
    double resolution_, resolution_inv_;
  };

  struct MappingData {
    std::vector<double> occupancy_buffer_;       // Occupancy probabilities (0: free, 1: occupied)
    std::vector<char> occupancy_buffer_neg_;     // Negative occupancy for free space
    std::vector<double> distance_buffer_;        // Positive distance field
    std::vector<double> distance_buffer_neg_;    // Negative distance field
    std::vector<double> distance_buffer_all_;    // Combined signed distance field
    std::vector<double> tmp_buffer1_;            // Temporary buffer for distance transform
    Eigen::Vector2i local_bound_min_, local_bound_max_; // Local update bounds
  };

  MappingParameters mp_;
  MappingData md_;

  // Core distance transform function
  template <typename F_get_val, typename F_set_val>
  void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim);

  // Utility functions
  inline void posToIndex(const Eigen::Vector2d& pos, Eigen::Vector2i& id);
  inline void indexToPos(const Eigen::Vector2i& id, Eigen::Vector2d& pos);
  inline int toAddress(const Eigen::Vector2i& id);
  inline bool isInMap(const Eigen::Vector2d& pos);
  inline void boundIndex(Eigen::Vector2i& id);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/* Inline utility functions */
inline void SDFMap2D::posToIndex(const Eigen::Vector2d& pos, Eigen::Vector2i& id) {
  for (int i = 0; i < 2; ++i) {
    id(i) = floor((pos(i) - mp_.map_origin_(i)) * mp_.resolution_inv_);
  }
}

inline void SDFMap2D::indexToPos(const Eigen::Vector2i& id, Eigen::Vector2d& pos) {
  for (int i = 0; i < 2; ++i) {
    pos(i) = (id(i) + 0.5) * mp_.resolution_ + mp_.map_origin_(i);
  }
}

inline int SDFMap2D::toAddress(const Eigen::Vector2i& id) {
  return id(0) * mp_.map_voxel_num_(1) + id(1);
}

inline bool SDFMap2D::isInMap(const Eigen::Vector2d& pos) {
  return pos(0) >= mp_.map_min_boundary_(0) + 1e-4 &&
        pos(1) >= mp_.map_min_boundary_(1) + 1e-4 &&
        pos(0) <= mp_.map_max_boundary_(0) - 1e-4 &&
        pos(1) <= mp_.map_max_boundary_(1) - 1e-4;
}

inline void SDFMap2D::boundIndex(Eigen::Vector2i& id) {
  id(0) = std::max(std::min(id(0), mp_.map_voxel_num_(0) - 1), 0);
  id(1) = std::max(std::min(id(1), mp_.map_voxel_num_(1) - 1), 0);
}

#endif