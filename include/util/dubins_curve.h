#ifndef DUBINS_CURVE_NEW_H
#define DUBINS_CURVE_NEW_H

#include <Eigen/Eigen>
#include <array>
#include <string>
#include <vector>

struct DubinsPoint {
  Eigen::Vector2d pos;
  double yaw;
};

struct DubinsPath {
  std::vector<Eigen::Vector2d> points;
  double length;
  std::array<std::string, 3> types; // e.g., {"left", "straight", "left"}
};

class DubinsCurveNew
{
public:
  DubinsCurveNew(double turning_radius, double sample_interval);
  DubinsPath computeDubinsPath(const DubinsPoint& start, const DubinsPoint& end, double radius);

private:
  std::array<DubinsPath, 4> computeAllPaths(const DubinsPoint& start, const DubinsPoint& end, double radius);
  DubinsPath computeLSL(const DubinsPoint& start, const DubinsPoint& end, double radius);
  DubinsPath computeLSR(const DubinsPoint& start, const DubinsPoint& end, double radius);
  DubinsPath computeRSR(const DubinsPoint& start, const DubinsPoint& end, double radius);
  DubinsPath computeRSL(const DubinsPoint& start, const DubinsPoint& end, double radius);
  void generateArcPoints(std::vector<Eigen::Vector2d>& points, const Eigen::Vector2d& center, 
                         double radius, double start_angle, double end_angle, 
                         const std::string& dir, double sample_interval);
  void generateStraightPoints(std::vector<Eigen::Vector2d>& points, 
                              const Eigen::Vector2d& start, const Eigen::Vector2d& end, 
                              double sample_interval);
  double normalizeAngle(double angle);

  double turning_radius_;
  double sample_interval_;
};

#endif // DUBINS_CURVE_NEW_H