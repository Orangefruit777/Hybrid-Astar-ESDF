#include "hybrid_a_star/quintic_spline.h"

VectorVec3d QuinticSpline::fitQuinticSpline(const VectorVec3d& path, 
                                            double dt, 
                                            double v,
                                            // const Eigen::Vector2d& start_vel,
                                            // const Eigen::Vector2d& end_vel,
                                            const Eigen::Vector2d& start_acc,
                                            const Eigen::Vector2d& end_acc) {
    if (path.size() < 2) {
        throw std::invalid_argument("At least two points are required for spline interpolation.");
    }

    // 从起点和终点的偏航角计算速度矢量
    double start_yaw = path[0].z();
    double end_yaw = path.back().z();
    
    Eigen::Vector2d start_vel;
    start_vel.x() = v * std::cos(start_yaw);
    start_vel.y() = v * std::sin(start_yaw);
    
    Eigen::Vector2d end_vel;
    end_vel.x() = v * std::cos(end_yaw);
    end_vel.y() = v * std::sin(end_yaw);

    // 分段数量
    size_t n = path.size() - 1; // n 段多项式
    std::vector<double> times(n + 1, 0.0); // 时间参数
    VectorVec3d spline_points;

    // 参数化时间（基于弧长）
    for (size_t i = 1; i <= n; ++i) {
        double dx = path[i].x() - path[i - 1].x();
        double dy = path[i].y() - path[i - 1].y();
        times[i] = times[i - 1] + std::sqrt(dx * dx + dy * dy) / v;
    }

    // 为 x 和 y 分别计算五阶多项式
    VectorVec6d x_coeffs = computeQuinticCoefficients(path, times, 0, 
                                                      start_vel.x(), end_vel.x(),
                                                      start_acc.x(), end_acc.x());
    VectorVec6d y_coeffs = computeQuinticCoefficients(path, times, 1,
                                                      start_vel.y(), end_vel.y(),
                                                      start_acc.y(), end_acc.y());

    // 采样平滑轨迹
    for (size_t i = 0; i < n; ++i) {
        double t_start = times[i];
        double t_end = times[i + 1];
        double T = t_end - t_start;
        
        for (double t = t_start; t < t_end; t += dt) {
            double u = (t - t_start) / T; // 归一化参数
            Eigen::Vector3d point;
            
            // 计算 x(t)
            point.x() = evaluateQuinticPolynomial(u, x_coeffs[i]);
            // 计算 y(t)
            point.y() = evaluateQuinticPolynomial(u, y_coeffs[i]);
            
            // 计算 yaw(t) = atan2(y'(t), x'(t))
            double x_prime = evaluateQuinticDerivative(u, x_coeffs[i], T);
            double y_prime = evaluateQuinticDerivative(u, y_coeffs[i], T);
            point.z() = std::atan2(y_prime, x_prime);
            
            spline_points.push_back(point);
        }
    }

    // 确保包含最后一个点
    Eigen::Vector3d last_point = path.back();
    if (spline_points.empty() || 
        (spline_points.back() - last_point).norm() > 1e-6) {
        spline_points.push_back(last_point);
    }

    return spline_points;
}

VectorVec6d QuinticSpline::computeQuinticCoefficients(const VectorVec3d& path,
                                                      const std::vector<double>& times,
                                                      int coord,
                                                      double start_vel,
                                                      double end_vel,
                                                      double start_acc,
                                                      double end_acc) {
    size_t n = path.size() - 1; // 分段数
    VectorVec6d coeffs(n); // 每段的系数 [a0, a1, a2, a3, a4, a5]

    // 估计中间点的速度和加速度
    std::vector<double> velocities = estimateVelocities(path, times, coord);
    std::vector<double> accelerations = estimateAccelerations(path, times, coord);

    // 设置边界条件
    velocities[0] = start_vel;
    velocities[n] = end_vel;
    accelerations[0] = start_acc;
    accelerations[n] = end_acc;

    // 计算每段五阶多项式的系数
    for (size_t i = 0; i < n; ++i) {
        double p0 = path[i][coord];
        double p1 = path[i + 1][coord];
        double v0 = velocities[i];
        double v1 = velocities[i + 1];
        double a0 = accelerations[i];
        double a1 = accelerations[i + 1];
        double T = times[i + 1] - times[i];

        coeffs[i] = computeSingleSegmentCoeffs(p0, p1, v0, v1, a0, a1, T);
    }

    return coeffs;
}

Eigen::Matrix<double, 6, 1> QuinticSpline::computeSingleSegmentCoeffs(double p0, double p1,
                                                                        double v0, double v1,
                                                                        double a0, double a1,
                                                                        double T) {
    // 五阶多项式：p(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
    // 边界条件：
    // p(0) = p0, p(T) = p1
    // p'(0) = v0, p'(T) = v1
    // p''(0) = a0, p''(T) = a1
    
    Eigen::Matrix<double, 6, 1> coeffs;
    
    // 使用归一化时间 u = t/T，多项式变为：
    // p(u) = c0 + c1*u + c2*u^2 + c3*u^3 + c4*u^4 + c5*u^5
    
    double T2 = T * T;
    // double T3 = T2 * T;
    // double T4 = T3 * T;
    // double T5 = T4 * T;
    
    coeffs[0] = p0; // c0 = p0
    coeffs[1] = v0 * T; // c1 = v0 * T
    coeffs[2] = a0 * T2 / 2.0; // c2 = a0 * T^2 / 2
    
    // 求解 c3, c4, c5
    // 从边界条件：p(1) = p1, p'(1) = v1*T, p''(1) = a1*T^2
    double A = p1 - p0 - v0 * T - a0 * T2 / 2.0;
    double B = v1 * T - v0 * T - a0 * T2;
    double C = a1 * T2 - a0 * T2;
    
    // 解线性方程组
    coeffs[3] = 10.0 * A - 4.0 * B + 0.5 * C;
    coeffs[4] = -15.0 * A + 7.0 * B - C;
    coeffs[5] = 6.0 * A - 3.0 * B + 0.5 * C;
    
    return coeffs;
}

double QuinticSpline::evaluateQuinticPolynomial(double u, const Eigen::Matrix<double, 6, 1>& coeffs) {
    // 使用Horner方法计算多项式值
    return coeffs[0] + u * (coeffs[1] + u * (coeffs[2] + u * (coeffs[3] + u * (coeffs[4] + u * coeffs[5]))));
}

double QuinticSpline::evaluateQuinticDerivative(double u, const Eigen::Matrix<double, 6, 1>& coeffs, double T) {
    // 计算归一化导数，然后转换为实际导数
    double du_dt = 1.0 / T;
    double poly_derivative = coeffs[1] + u * (2.0 * coeffs[2] + u * (3.0 * coeffs[3] + u * (4.0 * coeffs[4] + 5.0 * u * coeffs[5])));
    return poly_derivative * du_dt;
}

std::vector<double> QuinticSpline::estimateVelocities(const VectorVec3d& path,
                                                      const std::vector<double>& times,
                                                      int coord) {
    size_t n = path.size();
    std::vector<double> velocities(n, 0.0);
    
    // 使用中心差分估计速度
    for (size_t i = 1; i < n - 1; ++i) {
        double dt1 = times[i] - times[i - 1];
        double dt2 = times[i + 1] - times[i];
        double dp1 = path[i][coord] - path[i - 1][coord];
        double dp2 = path[i + 1][coord] - path[i][coord];
        
        // 加权平均
        velocities[i] = (dp1 / dt1 + dp2 / dt2) / 2.0;
    }
    
    // 边界点使用前向/后向差分
    if (n > 1) {
        velocities[0] = (path[1][coord] - path[0][coord]) / (times[1] - times[0]);
        velocities[n - 1] = (path[n - 1][coord] - path[n - 2][coord]) / (times[n - 1] - times[n - 2]);
    }
    
    return velocities;
}

std::vector<double> QuinticSpline::estimateAccelerations(const VectorVec3d& path,
                                                         const std::vector<double>& times,
                                                         int coord) {
    size_t n = path.size();
    std::vector<double> accelerations(n, 0.0);
    
    // 先估计速度
    std::vector<double> velocities = estimateVelocities(path, times, coord);
    
    // 使用中心差分估计加速度
    for (size_t i = 1; i < n - 1; ++i) {
        double dt = (times[i + 1] - times[i - 1]) / 2.0;
        double dv = velocities[i + 1] - velocities[i - 1];
        accelerations[i] = dv / (2.0 * dt);
    }
    
    // 边界点加速度设为0（或可根据需要调整）
    accelerations[0] = 0.0;
    accelerations[n - 1] = 0.0;
    
    return accelerations;
}