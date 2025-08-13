#include "hybrid_a_star/smooth.h"
Smoother::Smoother(){}

double Smoother::costFunction(void *ptr, 
                              const VecXd &x, 
                              VecXd &g) {
    auto instance = reinterpret_cast<Smoother *>(ptr);
    VectorVec3d smooth_path = instance->smooth_path_;
    const int points_num = smooth_path.size() - 4;
    Eigen::Matrix2Xd opt_points;
    opt_points.resize(2, smooth_path.size());
    opt_points(0,0) = smooth_path[0](0);
    opt_points(1,0) = smooth_path[0](1);
    opt_points(0,1) = smooth_path[1](0);
    opt_points(1,1) = smooth_path[1](1);
    
    opt_points.block(0,2,1,points_num) = x.head(points_num).transpose();
    opt_points.block(1,2,1,points_num) = x.tail(points_num).transpose();
    opt_points.col(smooth_path.size()-2)(0) = smooth_path[smooth_path.size()-2](0);
    opt_points.col(smooth_path.size()-2)(1) = smooth_path[smooth_path.size()-2](1);
    opt_points.col(smooth_path.size()-1)(0) = smooth_path[smooth_path.size()-1](0);
    opt_points.col(smooth_path.size()-1)(1) = smooth_path[smooth_path.size()-1](1);

    Eigen::Matrix2Xd grad;
    grad.resize(2, points_num);
    grad.setZero();
    double cost = 0.0;

    double max_clearance = instance->max_clearance_;
    
    // 碰撞代价
    double collision_cost = 0.0;
    Eigen::Matrix2Xd collision_grad;
    collision_grad.resize(2, points_num);
    collision_grad.setZero();

    for (int i = 2; i < opt_points.cols()-2; ++i) {
        Vec2d pos(opt_points(0, i), opt_points(1, i));
        double dist2obs;
        Vec2d gradient;
        instance->sdf_map_->evaluateEDTWithGrad(pos, dist2obs, gradient);

        if (dist2obs < max_clearance) {
            collision_cost += instance->w_obs_ * pow((dist2obs - max_clearance), 2);
            collision_grad.col(i-2) = instance->w_obs_ * 2 * (dist2obs - max_clearance) * gradient;
        } else {
            collision_cost += 0;
            collision_grad.col(i-2) = Vec2d::Zero();
        }
    }
    cost += collision_cost;
    grad += collision_grad;

    // 平滑代价
    double smooth_cost = 0.0;
    Eigen::Matrix2Xd smooth_grad;
    smooth_grad.resize(2, points_num);
    smooth_grad.setZero();
    for (int i = 2; i < opt_points.cols()-2; ++i) {
        Vec2d x_p2(opt_points(0, i-2), opt_points(1, i-2));
        Vec2d x_p(opt_points(0, i-1), opt_points(1, i-1));
        Vec2d x_c(opt_points(0, i), opt_points(1, i));
        Vec2d x_n(opt_points(0, i+1), opt_points(1, i+1));
        Vec2d x_n2(opt_points(0, i+2), opt_points(1, i+2));

        Vec2d err = x_p + x_n - 2 * x_c;
        smooth_cost += instance->w_smo_ * err.transpose() * err;
        smooth_grad.col(i-2) = instance->w_smo_ * 2 * (x_p2 - 4 * x_p + 6 * x_c - 4 * x_n + x_n2);
    }
    cost += smooth_cost;
    grad += smooth_grad;


    // // 3. 新增：偏航角平滑代价
    // double yaw_smooth_cost = 0.0;
    // Eigen::VectorXd yaw_smooth_grad;
    // yaw_smooth_grad.resize(points_num);
    // yaw_smooth_grad.setZero();

    // for (int i = 2; i < opt_points.cols()-2; ++i) {
    //     double yaw_p = opt_points(2, i-1);
    //     double yaw_c = opt_points(2, i);
    //     double yaw_n = opt_points(2, i+1);
        
    //     // 处理角度环绕
    //     double diff_p = normalizeAngle(yaw_c - yaw_p);
    //     double diff_n = normalizeAngle(yaw_n - yaw_c);
    //     double yaw_err = diff_p - diff_n;
        
    //     yaw_smooth_cost += instance->w_yaw_smooth_ * yaw_err * yaw_err;
    //     yaw_smooth_grad(i-2) = instance->w_yaw_smooth_ * 2.0 * yaw_err;
    // }
    // cost += yaw_smooth_cost;
    // grad.row(2) += yaw_smooth_grad.transpose();

    // // 4. 新增：偏航角一致性代价（偏航角应与运动方向一致）
    // double yaw_consistency_cost = 0.0;
    // Eigen::Matrix3Xd yaw_consistency_grad;
    // yaw_consistency_grad.resize(3, points_num);
    // yaw_consistency_grad.setZero();

    // for (int i = 2; i < opt_points.cols()-2; ++i) {
    //     if (i > 2 && i < opt_points.cols()-3) {
    //         // 计算运动方向
    //         Vec2d motion_dir(opt_points(0, i+1) - opt_points(0, i-1), 
    //                         opt_points(1, i+1) - opt_points(1, i-1));
    //         double motion_yaw = std::atan2(motion_dir.y(), motion_dir.x());
            
    //         // 计算偏航角与运动方向的偏差
    //         double yaw_diff = normalizeAngle(opt_points(2, i) - motion_yaw);
            
    //         yaw_consistency_cost += instance->w_yaw_consist_ * yaw_diff * yaw_diff;
            
    //         // 计算梯度
    //         yaw_consistency_grad(2, i-2) = instance->w_yaw_consist_ * 2.0 * yaw_diff;
            
    //         // 对位置的梯度（运动方向对位置的导数）
    //         double motion_dir_norm = motion_dir.norm();
    //         if (motion_dir_norm > 1e-6) {
    //             Vec2d d_motion_yaw_d_pos;
    //             d_motion_yaw_d_pos.x() = -motion_dir.y() / (motion_dir_norm * motion_dir_norm);
    //             d_motion_yaw_d_pos.y() = motion_dir.x() / (motion_dir_norm * motion_dir_norm);
                
    //             // 这里需要考虑对前后点位置的影响
    //             yaw_consistency_grad(0, i-2) += instance->w_yaw_consist_ * 2.0 * yaw_diff * (-d_motion_yaw_d_pos.x());
    //             yaw_consistency_grad(1, i-2) += instance->w_yaw_consist_ * 2.0 * yaw_diff * (-d_motion_yaw_d_pos.y());
    //         }
    //     }
    // }
    // cost += yaw_consistency_cost;
    // grad += yaw_consistency_grad;



    // 曲率代价
    double curvature_cost = 0.0;
    Eigen::Matrix2Xd curvature_grad;
    curvature_grad.resize(2, points_num);
    curvature_grad.setZero();
    for (int i = 2; i < opt_points.cols()-2; ++i) {
        Vec2d x_p2(opt_points(0, i-2), opt_points(1, i-2));
        Vec2d x_p(opt_points(0, i-1), opt_points(1, i-1));
        Vec2d x_c(opt_points(0, i), opt_points(1, i));
        Vec2d x_n(opt_points(0, i+1), opt_points(1, i+1));
        Vec2d x_n2(opt_points(0, i+2), opt_points(1, i+2));

        Vec2d delta_x_p = x_p - x_p2;
        Vec2d delta_x_c = x_c - x_p;
        Vec2d delta_x_n = x_n - x_c;
        Vec2d delta_x_n2 = x_n2 - x_n;

        if (delta_x_p.norm() > 0 && delta_x_c.norm() > 0 && delta_x_n.norm() > 0 && delta_x_n2.norm() > 0) {
            double delta_phi_p = std::acos(std::min(std::max(delta_x_p.dot(delta_x_c) / delta_x_p.norm() / delta_x_c.norm(), -1.0), 1.0));
            double delta_phi_c = std::acos(std::min(std::max(delta_x_c.dot(delta_x_n) / delta_x_c.norm() / delta_x_n.norm(), -1.0), 1.0));
            double delta_phi_n = std::acos(std::min(std::max(delta_x_n.dot(delta_x_n2) / delta_x_n.norm() / delta_x_n2.norm(), -1.0), 1.0));

            double kappa_p = delta_phi_p / delta_x_p.norm();
            double kappa_c = delta_phi_c / delta_x_c.norm();
            double kappa_n = delta_phi_n / delta_x_n.norm();

            if (kappa_c > instance->max_kappa_ && kappa_p > 0 && kappa_n > 0) {
                auto compute_d_delta_phi = [](const double delta_phi) {
                    return -1.0 / std::sqrt(1.0 - std::pow(std::cos(delta_phi), 2));
                };

                auto compute_orthogonal_complement = [](Vec2d x0, Vec2d x1) {
                    return x0 - x1 * x0.dot(x1) / std::pow(x1.norm(), 2);
                };

                double d_delta_phi_p = compute_d_delta_phi(delta_phi_p);
                Vec2d d_cos_delta_phi_p = compute_orthogonal_complement(delta_x_p, delta_x_c) 
                                        / delta_x_p.norm() / delta_x_c.norm();
                Vec2d d_kappa_p = 1.0 / delta_x_p.norm() * d_delta_phi_p * d_cos_delta_phi_p;
                Vec2d k_p = 2.0 * (kappa_p - instance->max_kappa_) * d_kappa_p;

                double d_delta_phi_c = compute_d_delta_phi(delta_phi_c);
                Vec2d d_cos_delta_phi_c = compute_orthogonal_complement(delta_x_n, delta_x_c) 
                                        / delta_x_c.norm() / delta_x_n.norm()
                                        - compute_orthogonal_complement(delta_x_c, delta_x_n)
                                        / delta_x_c.norm() / delta_x_n.norm();
                Vec2d d_kappa_c = 1.0 / delta_x_c.norm() * d_delta_phi_c * d_cos_delta_phi_c 
                                - delta_phi_c / std::pow(delta_x_c.norm(), 3) * delta_x_c;
                Vec2d k_c = 2.0 * (kappa_c - instance->max_kappa_) * d_kappa_c;

                double d_delta_phi_n = compute_d_delta_phi(delta_phi_n);
                Vec2d d_cos_delta_phi_n = -compute_orthogonal_complement(delta_x_n2, delta_x_n) 
                                        / delta_x_n.norm() / delta_x_n2.norm();
                Vec2d d_kappa_n = 1.0 / delta_x_n.norm() * d_delta_phi_n * d_cos_delta_phi_n 
                                + delta_phi_n / std::pow(delta_x_n.norm(), 3) * delta_x_n;
                Vec2d k_n = 2.0 * (kappa_n - instance->max_kappa_) * d_kappa_n;

                curvature_cost += instance->w_cur_ * std::pow(kappa_c - instance->max_kappa_, 2);
                curvature_grad.col(i-2) = instance->w_cur_ * (0.25 * k_p + 0.5 * k_c + 0.25 * k_n);
            } else {
                curvature_cost += 0;
                curvature_grad.col(i-2) = Vec2d(0, 0);
            }
        }
    }
    cost += curvature_cost;
    grad += curvature_grad;

    // 参考线距离代价
    double reference_cost = 0.0;
    Eigen::Matrix2Xd reference_grad;
    reference_grad.resize(2, points_num);
    reference_grad.setZero();

    for (int i = 2; i < opt_points.cols()-2; ++i) {
        Vec2d pos(opt_points(0, i), opt_points(1, i));
        double dist2ref;
        Vec2d gradient;
        instance->sdf_map_refer_->evaluateEDTWithGrad(pos, dist2ref, gradient);

        reference_cost += instance->w_ref_ * dist2ref * dist2ref;
        reference_grad.col(i-2) = instance->w_ref_ * 2 * dist2ref * gradient;
    }
    cost += reference_cost;
    grad += reference_grad;

    g.setZero();
    g.head(points_num) = grad.row(0).transpose();
    g.tail(points_num) = grad.row(1).transpose();

    return cost;
}

double Smoother::optimize(std::shared_ptr<SDFMap2D> sdf_map, std::shared_ptr<SDFMap2D> sdf_map_refer, VectorVec3d &path) {
    smooth_path_ = path;
    sdf_map_ = sdf_map;
    sdf_map_refer_ = sdf_map_refer;

    int points_num = smooth_path_.size() - 4;
    VecXd x(2 * points_num);
    for (int i = 2; i < static_cast<int>(smooth_path_.size())-2; ++i) {
        x(i-2) = smooth_path_[i](0);
        x(i-2 + points_num) = smooth_path_[i](1);
    }

    double minCost = 0.0;
    lbfgs_params.mem_size = 256;
    lbfgs_params.past = 3;
    lbfgs_params.min_step = 1.0e-32;
    lbfgs_params.g_epsilon = 0.0;
    lbfgs_params.delta = 1.0e-5;

    int ret = lbfgs::lbfgs_optimize(x,
                                    minCost,
                                    &Smoother::costFunction,
                                    nullptr,
                                    nullptr,
                                    this,
                                    lbfgs_params);
    
    for (int i = 2; i < static_cast<int>(smooth_path_.size())-2; ++i) {
        smooth_path_[i](0) = x(i-2);
        smooth_path_[i](1) = x(i-2 + points_num);
    }

    if (ret >= 0) {
        std::cout << "Optimization success" << std::endl;
    } else {
        minCost = INFINITY;
        std::cout << "Optimization Failed: "
                  << lbfgs::lbfgs_strerror(ret)
                  << std::endl;
    }

    return minCost;
}

// VectorVec3d Smoother::adjustPathYaw(const VectorVec3d& path) {
//     if (path.size() < 2) return path;
    
//     VectorVec3d adjusted_path = path;
    
//     // 调整中间点的偏航角
//     for (size_t i = 1; i < path.size() - 1; ++i) {
//         // 使用前后两点的连线方向计算偏航角
//         double dx = path[i + 1].x() - path[i - 1].x();
//         double dy = path[i + 1].y() - path[i - 1].y();
//         adjusted_path[i].z() = std::atan2(dy, dx);
//     }
    
//     // 处理角度跳跃，确保连续性
//     for (size_t i = 1; i < adjusted_path.size(); ++i) {
//         double diff = adjusted_path[i].z() - adjusted_path[i - 1].z();
//         if (diff > M_PI) {
//             adjusted_path[i].z() -= 2.0 * M_PI;
//         } else if (diff < -M_PI) {
//             adjusted_path[i].z() += 2.0 * M_PI;
//         }
//     }
    
//     return adjusted_path;
// }