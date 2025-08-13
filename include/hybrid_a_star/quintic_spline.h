#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <stdexcept>
#include "util/type.h"

// 类型定义（假设与原代码保持一致）
typedef std::vector<Eigen::Matrix<double, 6, 1>> VectorVec6d;

class QuinticSpline {
public:
    /**
     * 五阶多项式曲线拟合
     * @param path 输入路径点
     * @param dt 采样时间间隔
     * @param v 参考速度（用于参数化）
     * @param start_vel 起点速度 (默认为0)
     * @param end_vel 终点速度 (默认为0)
     * @param start_acc 起点加速度 (默认为0)
     * @param end_acc 终点加速度 (默认为0)
     * @return 插值后的平滑轨迹点
     */
    static VectorVec3d fitQuinticSpline(const VectorVec3d& path, 
                                        double dt, 
                                        double v,
                                        // const Eigen::Vector2d& start_vel = Eigen::Vector2d::Zero(),
                                        // const Eigen::Vector2d& end_vel = Eigen::Vector2d::Zero(),
                                        const Eigen::Vector2d& start_acc = Eigen::Vector2d::Zero(),
                                        const Eigen::Vector2d& end_acc = Eigen::Vector2d::Zero());

private:
    /**
     * 计算五阶多项式系数
     * @param path 路径点
     * @param times 时间参数
     * @param coord 坐标索引 (0: x, 1: y)
     * @param start_vel 起点速度
     * @param end_vel 终点速度
     * @param start_acc 起点加速度
     * @param end_acc 终点加速度
     * @return 每段的六个系数 [a0, a1, a2, a3, a4, a5]
     */
    static VectorVec6d computeQuinticCoefficients(const VectorVec3d& path,
                                                  const std::vector<double>& times,
                                                  int coord,
                                                  double start_vel,
                                                  double end_vel,
                                                  double start_acc,
                                                  double end_acc);

    /**
     * 计算单段五阶多项式系数
     * @param p0 起点位置
     * @param p1 终点位置
     * @param v0 起点速度
     * @param v1 终点速度
     * @param a0 起点加速度
     * @param a1 终点加速度
     * @param T 时间长度
     * @return 六个系数 [a0, a1, a2, a3, a4, a5]
     */
    static Eigen::Matrix<double, 6, 1> computeSingleSegmentCoeffs(double p0, double p1,
                                                                  double v0, double v1,
                                                                  double a0, double a1,
                                                                  double T);

    /**
     * 计算五阶多项式的值
     * @param u 归一化参数 [0, 1]
     * @param coeffs 六个系数
     * @return 多项式值
     */
    static double evaluateQuinticPolynomial(double u, const Eigen::Matrix<double, 6, 1>& coeffs);

    /**
     * 计算五阶多项式的一阶导数
     * @param u 归一化参数 [0, 1]
     * @param coeffs 六个系数
     * @param T 时间长度（用于归一化）
     * @return 一阶导数值
     */
    static double evaluateQuinticDerivative(double u, const Eigen::Matrix<double, 6, 1>& coeffs, double T);

    /**
     * 解五对角矩阵系统（用于连续性约束）
     * 这里简化处理，使用分段独立的方法
     */
    static std::vector<double> estimateVelocities(const VectorVec3d& path,
                                                  const std::vector<double>& times,
                                                  int coord);
    
    static std::vector<double> estimateAccelerations(const VectorVec3d& path,
                                                     const std::vector<double>& times,
                                                     int coord);
};