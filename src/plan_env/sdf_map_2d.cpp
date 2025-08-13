#include "plan_env/sdf_map_2d.h"
#include <limits>

void SDFMap2D::initMap(const Eigen::Vector2d& origin,  const Eigen::Vector2i& size, double resolution,
                      const std::vector<Eigen::Vector2d>& obstacles) {
  // Set map parameters
  mp_.map_voxel_num_ = size;
  mp_.resolution_ = resolution;
  mp_.resolution_inv_ = 1.0 / resolution;
  mp_.map_size_ = mp_.resolution_ * size.cast<double>();
  mp_.map_origin_ = origin;
  // mp_.map_origin_ = Eigen::Vector2d(0.0, 0.0); // Origin at (0,0)
  mp_.map_min_boundary_ = mp_.map_origin_;
  mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;

  // Initialize buffers
  int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1);
  md_.occupancy_buffer_ = std::vector<double>(buffer_size, 0.0); // Default to free
  md_.occupancy_buffer_neg_ = std::vector<char>(buffer_size, 1); // Default to free
  md_.distance_buffer_ = std::vector<double>(buffer_size, std::numeric_limits<double>::max());
  md_.distance_buffer_neg_ = std::vector<double>(buffer_size, std::numeric_limits<double>::max());
  md_.distance_buffer_all_ = std::vector<double>(buffer_size, std::numeric_limits<double>::max());
  md_.tmp_buffer1_ = std::vector<double>(buffer_size, 0.0);

  // Set obstacles
  for (const auto& obs : obstacles) {
    if (isInMap(obs)) {
      Eigen::Vector2i id;
      posToIndex(obs, id);
      md_.occupancy_buffer_[toAddress(id)] = 1.0;
    }
  }

  // Set local bounds to cover entire map
  md_.local_bound_min_ = Eigen::Vector2i(0, 0);
  md_.local_bound_max_ = mp_.map_voxel_num_ - Eigen::Vector2i(1, 1);

  // Update ESDF
  updateESDF2d();
}

template <typename F_get_val, typename F_set_val>
void SDFMap2D::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim) {
  int v[mp_.map_voxel_num_(dim)];
  double z[mp_.map_voxel_num_(dim) + 1];

  int k = start;
  v[start] = start;
  z[start] = -std::numeric_limits<double>::max();
  z[start + 1] = std::numeric_limits<double>::max();

  for (int q = start + 1; q <= end; q++) {
    k++;
    double s;
    do {
      k--;
      s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
    } while (s <= z[k]);

    k++;
    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<double>::max();
  }

  k = start;
  for (int q = start; q <= end; q++) {
    while (z[k + 1] < q) k++;
    double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
    f_set_val(q, val);
  }
}

void SDFMap2D::updateESDF2d() {
  Eigen::Vector2i min_esdf = md_.local_bound_min_;
  Eigen::Vector2i max_esdf = md_.local_bound_max_;

  // Compute positive distance field
  for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
    fillESDF(
        [&](int y) {
          return md_.occupancy_buffer_[toAddress(Eigen::Vector2i(x, y))] == 1 ? 0 : std::numeric_limits<double>::max();
        },
        [&](int y, double val) { md_.tmp_buffer1_[toAddress(Eigen::Vector2i(x, y))] = val; },
        min_esdf[1], max_esdf[1], 1);
  }

  for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
    fillESDF(
        [&](int x) { return md_.tmp_buffer1_[toAddress(Eigen::Vector2i(x, y))]; },
        [&](int x, double val) {
          md_.distance_buffer_[toAddress(Eigen::Vector2i(x, y))] = mp_.resolution_ * std::sqrt(val);
        },
        min_esdf[0], max_esdf[0], 0);
  }

  // Compute negative distance field
  for (int x = min_esdf[0]; x <= max_esdf[0]; ++x) {
    for (int y = min_esdf[1]; y <= max_esdf[1]; ++y) {
      int idx = toAddress(Eigen::Vector2i(x, y));
      md_.occupancy_buffer_neg_[idx] = (md_.occupancy_buffer_[idx] == 0) ? 1 : 0;
    }
  }

  for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
    fillESDF(
        [&](int y) {
          return md_.occupancy_buffer_neg_[toAddress(Eigen::Vector2i(x, y))] == 1 ? 0 : std::numeric_limits<double>::max();
        },
        [&](int y, double val) { md_.tmp_buffer1_[toAddress(Eigen::Vector2i(x, y))] = val; },
        min_esdf[1], max_esdf[1], 1);
  }

  for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
    fillESDF(
        [&](int x) { return md_.tmp_buffer1_[toAddress(Eigen::Vector2i(x, y))]; },
        [&](int x, double val) {
          md_.distance_buffer_neg_[toAddress(Eigen::Vector2i(x, y))] = mp_.resolution_ * std::sqrt(val);
        },
        min_esdf[0], max_esdf[0], 0);
  }

  // Combine positive and negative distance fields
  for (int x = min_esdf[0]; x <= max_esdf[0]; ++x) {
    for (int y = min_esdf[1]; y <= max_esdf[1]; ++y) {
      int idx = toAddress(Eigen::Vector2i(x, y));
      md_.distance_buffer_all_[idx] = md_.distance_buffer_[idx];
      if (md_.distance_buffer_neg_[idx] > 0.0) {
        md_.distance_buffer_all_[idx] += (-md_.distance_buffer_neg_[idx] + mp_.resolution_);
      }
    }
  }
}

double SDFMap2D::getDistance(const Eigen::Vector2d& pos) {
  if (!isInMap(pos)) {
    std::cout << "Position out of map bounds." << std::endl;
    return 0.0;
  }

  Eigen::Vector2i id;
  posToIndex(pos, id);
  boundIndex(id);
  // 打印distance_buffer_all_大小
  // std::cout << "distance_buffer_all_ size: " << md_.distance_buffer_all_.size() << std::endl;
  return md_.distance_buffer_all_[toAddress(id)];
}

// 获取梯度
void SDFMap2D::getSurroundPts(const Eigen::Vector2d& pos, Eigen::Vector2d pts[2][2], Eigen::Vector2d& diff) {
  if (!isInMap(pos)) {
    // Warning: pos invalid for interpolation
  }

  /* interpolation position */
  Eigen::Vector2d pos_m = pos - 0.5 * mp_.resolution_ * Eigen::Vector2d::Ones();
  Eigen::Vector2i idx;
  Eigen::Vector2d idx_pos;

  posToIndex(pos_m, idx);
  indexToPos(idx, idx_pos);
  diff = (pos - idx_pos) * mp_.resolution_inv_;

  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
      Eigen::Vector2i current_idx = idx + Eigen::Vector2i(x, y);
      Eigen::Vector2d current_pos;
      indexToPos(current_idx, current_pos);
      pts[x][y] = current_pos;
    }
  }
}

void SDFMap2D::interpolateBilinear(double values[2][2], const Eigen::Vector2d& diff, double& value, Eigen::Vector2d& grad) {
  // Bilinear interpolation
  double v0 = (1 - diff(0)) * values[0][0] + diff(0) * values[1][0];
  double v1 = (1 - diff(0)) * values[0][1] + diff(0) * values[1][1];
  value = (1 - diff(1)) * v0 + diff(1) * v1;

  grad[1] = (v1 - v0) * mp_.resolution_inv_;
  grad[0] = (1 - diff[1]) * (values[1][0] - values[0][0]) + diff[1] * (values[1][1] - values[0][1]);
  grad[0] *= mp_.resolution_inv_;
}

void SDFMap2D::getSurroundDistance(Eigen::Vector2d pts[2][2], double dists[2][2]) {
  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
      dists[x][y] = getDistance(pts[x][y]);
    }
  }
}

void SDFMap2D::evaluateEDTWithGrad(const Eigen::Vector2d& pos, double& dist, Eigen::Vector2d& grad) {
  Eigen::Vector2d diff;
  Eigen::Vector2d sur_pts[2][2];
  getSurroundPts(pos, sur_pts, diff);

  double dists[2][2];
  getSurroundDistance(sur_pts, dists);

  interpolateBilinear(dists, diff, dist, grad);
}