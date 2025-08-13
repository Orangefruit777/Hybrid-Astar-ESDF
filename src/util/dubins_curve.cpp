#include "util/dubins_curve.h"
#include <cmath>

DubinsCurveNew::DubinsCurveNew(double turning_radius, double sample_interval)
    : turning_radius_(turning_radius), sample_interval_(sample_interval)
{
}

double DubinsCurveNew::normalizeAngle(double angle)
{
  while (angle > M_PI) angle -= 2 * M_PI;
  while (angle < -M_PI) angle += 2 * M_PI;
  return angle;
}

DubinsPath DubinsCurveNew::computeDubinsPath(const DubinsPoint& start, const DubinsPoint& end, double radius)
{
  auto paths = computeAllPaths(start, end, radius);
  int best_idx = 0;
  double min_length = std::numeric_limits<double>::infinity();
  for (int i = 0; i < 4; ++i) {
    if (paths[i].length < min_length) {
      min_length = paths[i].length;
      best_idx = i;
    }
  }
  return paths[best_idx];
}

std::array<DubinsPath, 4> DubinsCurveNew::computeAllPaths(const DubinsPoint& start, 
                                                         const DubinsPoint& end, 
                                                         double radius)
{
  return {
    computeLSL(start, end, radius),
    computeLSR(start, end, radius),
    computeRSR(start, end, radius),
    computeRSL(start, end, radius)
  };
}

DubinsPath DubinsCurveNew::computeLSL(const DubinsPoint& start, const DubinsPoint& end, double radius)
{
  DubinsPath path;
  path.types = {"left", "straight", "left"};

  Eigen::Vector2d c1(start.pos.x() + radius * std::cos(start.yaw + M_PI_2), 
                     start.pos.y() + radius * std::sin(start.yaw + M_PI_2));
  Eigen::Vector2d c2(end.pos.x() + radius * std::cos(end.yaw + M_PI_2), 
                     end.pos.y() + radius * std::sin(end.yaw + M_PI_2));
  Eigen::Vector2d center_line = c2 - c1;
  // double dist_centers = center_line.norm();
  double theta = std::atan2(center_line.y(), center_line.x()) - M_PI_2;

  Eigen::Vector2d t1 = c1 + radius * Eigen::Vector2d(std::cos(theta), std::sin(theta));
  Eigen::Vector2d t2 = c2 + radius * Eigen::Vector2d(std::cos(theta), std::sin(theta));

  double start_angle = std::atan2(start.pos.y() - c1.y(), start.pos.x() - c1.x());
  double t1_angle = std::atan2(t1.y() - c1.y(), t1.x() - c1.x());
  double delta1 = normalizeAngle(t1_angle - start_angle);
  if (delta1 < 0) delta1 += 2 * M_PI;

  double t2_angle = std::atan2(t2.y() - c2.y(), t2.x() - c2.x());
  double end_angle = std::atan2(end.pos.y() - c2.y(), end.pos.x() - c2.x());
  double delta2 = normalizeAngle(end_angle - t2_angle);
  if (delta2 < 0) delta2 += 2 * M_PI;

  path.length = radius * (std::abs(delta1) + std::abs(delta2)) + (t2 - t1).norm();

  generateArcPoints(path.points, c1, radius, start_angle, t1_angle, "left", sample_interval_);
  generateStraightPoints(path.points, t1, t2, sample_interval_);
  generateArcPoints(path.points, c2, radius, t2_angle, end_angle, "left", sample_interval_);

  return path;
}

DubinsPath DubinsCurveNew::computeLSR(const DubinsPoint& start, const DubinsPoint& end, double radius)
{
  DubinsPath path;
  path.types = {"left", "straight", "right"};

  Eigen::Vector2d c1(start.pos.x() + radius * std::cos(start.yaw + M_PI_2), 
                     start.pos.y() + radius * std::sin(start.yaw + M_PI_2));
  Eigen::Vector2d c2(end.pos.x() + radius * std::cos(end.yaw - M_PI_2), 
                     end.pos.y() + radius * std::sin(end.yaw - M_PI_2));
  Eigen::Vector2d center_line = c2 - c1;
  double dist_centers = center_line.norm();
  if (dist_centers < 2 * radius) {
    path.length = std::numeric_limits<double>::infinity();
    return path;
  }

  double R = dist_centers / 2;
  double theta = std::atan2(center_line.y(), center_line.x()) - std::acos(radius / R);

  Eigen::Vector2d t1 = c1 + radius * Eigen::Vector2d(std::cos(theta), std::sin(theta));
  Eigen::Vector2d t2 = c2 + radius * Eigen::Vector2d(std::cos(theta + M_PI), std::sin(theta + M_PI));

  double start_angle = std::atan2(start.pos.y() - c1.y(), start.pos.x() - c1.x());
  double t1_angle = std::atan2(t1.y() - c1.y(), t1.x() - c1.x());
  double delta1 = normalizeAngle(t1_angle - start_angle);
  if (delta1 < 0) delta1 += 2 * M_PI;

  double t2_angle = std::atan2(t2.y() - c2.y(), t2.x() - c2.x());
  double end_angle = std::atan2(end.pos.y() - c2.y(), end.pos.x() - c2.x());
  double delta2 = normalizeAngle(t2_angle - end_angle);
  if (delta2 < 0) delta2 += 2 * M_PI;

  path.length = radius * (std::abs(delta1) + std::abs(delta2)) + (t2 - t1).norm();

  generateArcPoints(path.points, c1, radius, start_angle, t1_angle, "left", sample_interval_);
  generateStraightPoints(path.points, t1, t2, sample_interval_);
  generateArcPoints(path.points, c2, radius, t2_angle, end_angle, "right", sample_interval_);

  return path;
}

DubinsPath DubinsCurveNew::computeRSR(const DubinsPoint& start, const DubinsPoint& end, double radius)
{
  DubinsPath path;
  path.types = {"right", "straight", "right"};

  Eigen::Vector2d c1(start.pos.x() + radius * std::cos(start.yaw - M_PI_2), 
                     start.pos.y() + radius * std::sin(start.yaw - M_PI_2));
  Eigen::Vector2d c2(end.pos.x() + radius * std::cos(end.yaw - M_PI_2), 
                     end.pos.y() + radius * std::sin(end.yaw - M_PI_2));
  Eigen::Vector2d center_line = c2 - c1;
  // double dist_centers = center_line.norm();
  double theta = std::atan2(center_line.y(), center_line.x()) + M_PI_2;

  Eigen::Vector2d t1 = c1 + radius * Eigen::Vector2d(std::cos(theta), std::sin(theta));
  Eigen::Vector2d t2 = c2 + radius * Eigen::Vector2d(std::cos(theta), std::sin(theta));

  double start_angle = std::atan2(start.pos.y() - c1.y(), start.pos.x() - c1.x());
  double t1_angle = std::atan2(t1.y() - c1.y(), t1.x() - c1.x());
  double delta1 = normalizeAngle(start_angle - t1_angle);
  if (delta1 < 0) delta1 += 2 * M_PI;

  double t2_angle = std::atan2(t2.y() - c2.y(), t2.x() - c2.x());
  double end_angle = std::atan2(end.pos.y() - c2.y(), end.pos.x() - c2.x());
  double delta2 = normalizeAngle(t2_angle - end_angle);
  if (delta2 < 0) delta2 += 2 * M_PI;

  path.length = radius * (std::abs(delta1) + std::abs(delta2)) + (t2 - t1).norm();

  generateArcPoints(path.points, c1, radius, start_angle, t1_angle, "right", sample_interval_);
  generateStraightPoints(path.points, t1, t2, sample_interval_);
  generateArcPoints(path.points, c2, radius, t2_angle, end_angle, "right", sample_interval_);

  return path;
}

DubinsPath DubinsCurveNew::computeRSL(const DubinsPoint& start, const DubinsPoint& end, double radius)
{
  DubinsPath path;
  path.types = {"right", "straight", "left"};

  Eigen::Vector2d c1(start.pos.x() + radius * std::cos(start.yaw - M_PI_2), 
                     start.pos.y() + radius * std::sin(start.yaw - M_PI_2));
  Eigen::Vector2d c2(end.pos.x() + radius * std::cos(end.yaw + M_PI_2), 
                     end.pos.y() + radius * std::sin(end.yaw + M_PI_2));
  Eigen::Vector2d center_line = c2 - c1;
  double dist_centers = center_line.norm();
  if (dist_centers < 2 * radius) {
    path.length = std::numeric_limits<double>::infinity();
    return path;
  }

  double R = dist_centers / 2;
  double theta = std::atan2(center_line.y(), center_line.x()) + std::acos(radius / R);

  Eigen::Vector2d t1 = c1 + radius * Eigen::Vector2d(std::cos(theta), std::sin(theta));
  Eigen::Vector2d t2 = c2 + radius * Eigen::Vector2d(std::cos(theta + M_PI), std::sin(theta + M_PI));

  double start_angle = std::atan2(start.pos.y() - c1.y(), start.pos.x() - c1.x());
  double t1_angle = std::atan2(t1.y() - c1.y(), t1.x() - c1.x());
  double delta1 = normalizeAngle(start_angle - t1_angle);
  if (delta1 < 0) delta1 += 2 * M_PI;

  double t2_angle = std::atan2(t2.y() - c2.y(), t2.x() - c2.x());
  double end_angle = std::atan2(end.pos.y() - c2.y(), end.pos.x() - c2.x());
  double delta2 = normalizeAngle(end_angle - t2_angle);
  if (delta2 < 0) delta2 += 2 * M_PI;

  path.length = radius * (std::abs(delta1) + std::abs(delta2)) + (t2 - t1).norm();

  generateArcPoints(path.points, c1, radius, start_angle, t1_angle, "right", sample_interval_);
  generateStraightPoints(path.points, t1, t2, sample_interval_);
  generateArcPoints(path.points, c2, radius, t2_angle, end_angle, "left", sample_interval_);

  return path;
}

void DubinsCurveNew::generateArcPoints(std::vector<Eigen::Vector2d>& points, 
                                      const Eigen::Vector2d& center, double radius, 
                                      double start_angle, double end_angle, 
                                      const std::string& dir, double sample_interval)
{
  double delta = normalizeAngle(end_angle - start_angle);
  if (dir == "left" && delta < 0) delta += 2 * M_PI;
  else if (dir == "right" && delta > 0) delta -= 2 * M_PI;
  double arc_length = radius * std::abs(delta);
  int num_points = std::ceil(arc_length / sample_interval) + 1;

  for (int i = 0; i <= num_points; ++i) {
    double t = static_cast<double>(i) / num_points;
    double angle = start_angle + t * delta;
    Eigen::Vector2d pt(center.x() + radius * std::cos(angle), 
                       center.y() + radius * std::sin(angle));
    points.push_back(pt);
  }
}

void DubinsCurveNew::generateStraightPoints(std::vector<Eigen::Vector2d>& points, 
                                           const Eigen::Vector2d& start, 
                                           const Eigen::Vector2d& end, 
                                           double sample_interval)
{
  double dist = (end - start).norm();
  int num_points = std::ceil(dist / sample_interval) + 1;

  for (int i = 0; i <= num_points; ++i) {
    double t = static_cast<double>(i) / num_points;
    Eigen::Vector2d pt = start + t * (end - start);
    points.push_back(pt);
  }
}