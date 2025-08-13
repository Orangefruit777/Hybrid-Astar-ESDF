#pragma once

#include <cmath>
#include <vector>
#include <memory>
#include <iostream>
#include "plan_env/sdf_map_2d.h"
// #include "hybrid_a_star/hybrid_astar.h"
#include "util/type.h"
#include "util/lbfgs.hpp"
#include <cfloat>
#include <iomanip>

class Smoother
{
private:
    double max_kappa_ = 1.0 / 5;
    double max_clearance_ = 1.5;
    double w_obs_ = 1.0;
    double w_smo_ = 2.0;
    double w_cur_ = 0.1;
    double w_ref_ = 0.0;

    std::shared_ptr<SDFMap2D> sdf_map_;
    std::shared_ptr<SDFMap2D> sdf_map_refer_;
    
    VectorVec3d path_;
    VectorVec3d smooth_path_;

    lbfgs::lbfgs_parameter_t lbfgs_params;
    static double costFunction(void *ptr, const VecXd &x, VecXd &g);

public:
    Smoother();
    double optimize(std::shared_ptr<SDFMap2D> sdf_map, std::shared_ptr<SDFMap2D> sdf_map_refer, VectorVec3d &path);
    void getSmoothPath(VectorVec3d &smooth_path) {
        // smooth_path_ = adjustPathYaw(smooth_path_);
        smooth_path = smooth_path_; 
    }

    // VectorVec3d adjustPathYaw(const VectorVec3d& path);
    void set_w_ref(const double w_ref){
        w_ref_ = w_ref;
    }
};