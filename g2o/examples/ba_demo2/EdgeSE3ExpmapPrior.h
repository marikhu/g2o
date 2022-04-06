/**
 * This file is part of se2clam.
 * Copyright (C) Fan ZHENG (github.com/izhengfan), Hengbo TANG (github.com/hbtang)
 */
#ifndef EDGESE3EXPMAPPRIOR_H_
#define EDGESE3EXPMAPPRIOR_H_

#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/structure_only/structure_only_solver.h"
#include "g2o/stuff/sampler.h"
#include "g2o/types/sba/types_six_dof_expmap.h"

#include <g2o/core/eigen_types.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/slam3d/dquat2mat.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/factory.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
using namespace cv;

//#include "dep.h"

typedef Eigen::Matrix<double, 6,6> Matrix6d;
g2o::SE3Quat toSE3Quat(const cv::Mat &cvT);
Matrix6d toMatrix6d(const cv::Mat &cvMat6d);

class G2O_TYPES_SBA_API EdgeSE3ExpmapPrior: public g2o::BaseUnaryEdge<6, g2o::SE3Quat, g2o::VertexSE3Expmap> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSE3ExpmapPrior();

    // Useless functions we don't care
    virtual bool read(std::istream &is);
    virtual bool write(std::ostream &os) const;

    void computeError();

    void setMeasurement(const g2o::SE3Quat& m);

    virtual void linearizeOplus();
};


EdgeSE3ExpmapPrior* addPlaneMotionSE3Expmap(
    g2o::SparseOptimizer &opt, const g2o::SE3Quat &pose, int vId, const cv::Mat &extPara);
    
#endif // EDGESE3EXPMAPPRIOR_H_