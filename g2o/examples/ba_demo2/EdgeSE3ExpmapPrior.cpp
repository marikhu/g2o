#include "EdgeSE3ExpmapPrior.h"

EdgeSE3ExpmapPrior::EdgeSE3ExpmapPrior() : g2o::BaseUnaryEdge<6, g2o::SE3Quat, g2o::VertexSE3Expmap>() {
    setMeasurement(g2o::SE3Quat());
    information().setIdentity();
}

// Here, measurement = Tcw, also v->estimate is also Tcw
// http://jamessjackson.com/lie_algebra_tutorial/08-computing_the_adjoint/
void EdgeSE3ExpmapPrior::computeError(){
    g2o::VertexSE3Expmap *v = static_cast<g2o::VertexSE3Expmap*>(_vertices[0]);  // Pose optimized, Tcw 
    g2o::SE3Quat err = _measurement * v->estimate().inverse();  // _measurement is what was set, Tcw 
    //std::cout << "_measurement: " << _measurement << std::endl;
    //std::cout << "v->estimate(): " << v->estimate() << std::endl;
    //std::cout << "v->estimate().inverse(): " << v->estimate().inverse() << std::endl;
    //std::cout << "err = _measurement * v->estimate().inverse(): " << err << std::endl;
//    _error = _measurement.log() - v->estimate().log();
//    SE3Quat err = v->estimate().inverse() * _measurement;
//    SE3Quat err = _measurementInverse * v->estimate();
//    Eigen::AngleAxisd err_angleaxis(err.rotation());
//    _error.head<3>() = err_angleaxis.angle() * err_angleaxis.axis();
    _error = err.log();  // transform into trf mat -- Lie algebra, log(err)
    //std::cout << "_error = err.log(): " <<_error << std::endl;
//    _error.tail<3>() = err.translation();
}

void EdgeSE3ExpmapPrior::setMeasurement(const g2o::SE3Quat &m) {
    _measurement = m;
//    _measurementInverse = m.inverse();
//    _measurementInverseAdj = _measurementInverse.adj();
}

void EdgeSE3ExpmapPrior::linearizeOplus(){
//    VertexSE3Expmap *v = static_cast<VertexSE3Expmap*>(_vertices[0]);
//    Vector6d err = ( _measurement * v->estimate().inverse() ).log() ;
//    _jacobianOplusXi = -invJJl(-err);
//    _jacobianOplusXi = - _measurementInverseAdj;
    _jacobianOplusXi = - Matrix6d::Identity();
//    _jacobianOplusXi = _measurementInverseAdj;
}

bool EdgeSE3ExpmapPrior::read(std::istream &is){
    return true; 
}

bool EdgeSE3ExpmapPrior::write(std::ostream &os) const{
    return true;
} 



g2o::SE3Quat toSE3Quat(const cv::Mat &cvT)
{
    Eigen::Matrix<double,3,3> R;
    R << cvT.at<double>(0,0), cvT.at<double>(0,1), cvT.at<double>(0,2),
            cvT.at<double>(1,0), cvT.at<double>(1,1), cvT.at<double>(1,2),
            cvT.at<double>(2,0), cvT.at<double>(2,1), cvT.at<double>(2,2);

    Eigen::Matrix<double,3,1> t(cvT.at<double>(0,3), cvT.at<double>(1,3), cvT.at<double>(2,3));

    return g2o::SE3Quat(R,t);
}

Matrix6d toMatrix6d(const cv::Mat &cvMat6d)
{
    Matrix6d m = Matrix6d::Zero();
    for(int i = 0; i < 6; i++)
        for(int j = 0; j < 6; j++)
            m(i,j) = cvMat6d.at<double>(i,j);
    return m;
}


cv::Mat toCvMat(const g2o::Isometry3 &t)
{
    std::cout << "g2o::internal::toSE3Quat(t): " << g2o::internal::toSE3Quat(t) << std::endl;
    return toCvMat(g2o::internal::toSE3Quat(t));
}

cv::Mat toCvMat(const g2o::SE3Quat &SE3)
{
    Eigen::Matrix<double,4,4> eigMat = SE3.to_homogeneous_matrix();
    return toCvMat(eigMat);
}


cv::Mat toCvMat(const Eigen::Matrix<double,4,4> &m)
{
    cv::Mat cvMat(4,4,CV_64FC1);
    for(int i=0;i<4;i++)
        for(int j=0; j<4; j++)
            cvMat.at<double>(i,j)=m(i,j);

    return cvMat.clone();
}

Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat &cvMat3)
{
    // std::cout << "cvMat3: " << cvMat3 << std::endl;
    // std::cout << "cvMat3.type(): " << cvMat3.type() << std::endl;
    // std::cout << "cvMat3.at<double>(0,0): " << cvMat3.at<double>(0,0) << std::endl;
    // std::cout << "cvMat3.at<float>(0,0): " << cvMat3.at<float>(0,0) << std::endl;
    Eigen::Matrix<double,3,3> M;
    M << cvMat3.at<double>(0,0), cvMat3.at<double>(0,1), cvMat3.at<double>(0,2),
         cvMat3.at<double>(1,0), cvMat3.at<double>(1,1), cvMat3.at<double>(1,2),
         cvMat3.at<double>(2,0), cvMat3.at<double>(2,1), cvMat3.at<double>(2,2);

    return M;
}

EdgeSE3ExpmapPrior* addPlaneMotionSE3Expmap(
    g2o::SparseOptimizer &opt, const g2o::SE3Quat &pose, int vId, const cv::Mat &matT_CwrtW_init_fixed, bool bDebug)
{
#ifdef USE_EULER
    const cv::Mat Twc_fixed = matT_CwrtW_init_fixed;  // T_CwrtW = (pose0)^-1  = (T_WwrtC)^-1
    const cv::Mat Tcw_fixed = Twc_fixed.inv(); // T_CwrtW, in order to work on T_WwrtW where constraints are to be enforced.

    cv::Mat Tcw = toCvMat(pose);    // T WwrtC pose_i
    cv::Mat Tww = Twc_fixed * Tcw;  // This is the transformation of the 3D points in world frame
    // Here, Tww is where we enforce the SE2 constraints

    Mat matR(3,3,CV_64FC1);
    Tww(Rect(0,0,3,3)).copyTo(matR);
    // NOTE:  Tbw.rowRange(0,3).colRange(0,3) returns a matrix of type CV_32FC1
    // g2o::Vector3 euler = g2o::internal::toEuler(toMatrix3d(Tbw.rowRange(0,3).colRange(0,3)));
    // std::cout << "matR: " << matR << std::endl;
    // std::cout << "matR.type(): " << matR.type() << std::endl;
    // std::cout << "toMatrix3d(matR): " << toMatrix3d(matR) << std::endl;
    g2o::Vector3 euler = g2o::internal::toEuler(toMatrix3d(matR));
    double yaw = euler(2);

    // In world frame, fix pitch (Rx) and roll (Ry) to zero. Considering yaw (Rz) only 
    cv::Mat Rww = (cv::Mat_<double>(3,3) <<
                   cos(yaw), -sin(yaw), 0,
                   sin(yaw),  cos(yaw), 0,
                   0,         0,        1);
    Rww.copyTo(Tww(Rect(0,0,3,3)));
    Tww.at<double>(2,3) = 0; // Fix tz to 0

    if(bDebug)
    {
        std::cout << "Twc_fixed = matT_CwrtW_init_fixed:" << std::endl << Twc_fixed << std::endl;
        std::cout << "Tcw_fixed = bTc.inv():" << std::endl << Tcw_fixed << std::endl;
        std::cout << "Tcw = toCvMat(pose): " << Tcw << std::endl;
        std::cout << "Twc_fixed.type: " << Twc_fixed.type() << std::endl;
        std::cout << "Tcw.type: " << Tcw.type() << std::endl;
        std::cout << "Tww.type: " << Tww.type() << std::endl;
        std::cout << "Tww = Twc_fixed * Tcw: " << Tww << std::endl;
        std::cout << "euler: " << euler << std::endl;
        std::cout << "yaw: " << yaw << std::endl;
        std::cout << "Rww: " << Rww << std::endl;
    }
    std::cout << "Tww constrained: " << Tww << std::endl;

    // After constraining Tww, we update Tcw to what we expect, hence it is measurement      
    Tcw = Tcw_fixed * Tww;  
    
    //! Vector order: [rot, trans] == Rx, Ry, Rz, tx, ty, tz
    // Info_ww = Hessian matrix, i.e. the inverse of covariance matrix (uncertainty)
    // High values for sigma_Rx*sigma_Rx means low variance for Rx
    // Low values for sigma_tx*sigma_tx means high variance for tx
    // 1 meter for sigma_tz*sigma_tz means some variation allowed for tz
    Matrix6d Info_ww = Matrix6d::Zero();    // 6x6 matrix
    Info_ww(0,0) = 1e6;     // Rx
    Info_ww(1,1) = 1e6;     // Ry
    Info_ww(2,2) = 1e-4;    // Rz
    Info_ww(3,3) = 1e-4;    // tx, better with 1e-1 but could be data dependent, known/prior info
    Info_ww(4,4) = 1e-4;    // ty
    Info_ww(5,5) = 1;       // tz   // Allowing some perturbations

    if(bDebug)
    {
        std::cout << "Info_ww: " << Info_ww << std::endl;
        std::cout << "Tcw = Tcw_fixed * Tww: " << Tcw << std::endl;
    }   

    // Transfer of covariance (Hessian in this case)

    if(bDebug)
    {
        std::cout << "Twc_fixed: " << std::endl << Twc_fixed << std::endl;
        std::cout << "toSE3Quat(Twc_fixed): " << std::endl << toSE3Quat(Twc_fixed) << std::endl;
        // Adjoint (Adjugate) matrix for SE(3) used as Jacobian matrix
        std::cout << "toSE3Quat(Twc_fixed).adj(): " << std::endl << toSE3Quat(Twc_fixed).adj() << std::endl;
    }
    // Compute Jacobian of the Twc transformation
    Matrix6d J_ww_cc = toSE3Quat(Twc_fixed).adj();  // Twc_fixed -- pose 0 inv --> Jacobian for camera to world
    if(bDebug) std::cout << "J_ww_cc: " << std::endl << J_ww_cc << std::endl;
    Matrix6d Info_cc = J_ww_cc.transpose() * Info_ww * J_ww_cc;
    if(bDebug) std::cout << "Info_cc: " << std:: endl << Info_cc << std::endl;
#else

    g2o::SE3Quat Twc_fixed = toSE3Quat(matT_CwrtW_init_fixed);
    g2o::SE3Quat Tww = Twc_fixed * pose;

    Eigen::AngleAxisd AngleAxis_ww(Tww.rotation());
    Eigen::Vector3d Log_Rww = AngleAxis_ww.angle() * AngleAxis_ww.axis();
    AngleAxis_ww = Eigen::AngleAxisd(Log_Rww[2], Eigen::Vector3d::UnitZ());
    Tww.setRotation(Eigen::Quaterniond(AngleAxis_ww));

    Eigen::Vector3d xyz_ww = Tww.translation();
    xyz_ww[2] = 0;  // tz
    Tww.setTranslation(xyz_ww);

    g2o::SE3Quat Tcw = Twc_fixed.inverse() * Tww;

    //! Vector order: [rot, trans]
    Matrix6d Info_ww = Matrix6d::Zero();
    // Information is Hessian matrix, i.e. inverse of covariance matrix
    Info_ww(0,0) = 1e6;     // Rx
    Info_ww(1,1) = 1e6;     // Ry
    Info_ww(2,2) = 1e-4;    // Rz    
    Info_ww(3,3) = 1e-4;    // tx    
    Info_ww(4,4) = 1e-4;    // ty
    Info_ww(5,5) = 1;       // tz
    Matrix6d J_ww_cc = Twc_fixed.adj();
    Matrix6d Info_cc = J_ww_cc.transpose() * Info_ww * J_ww_cc;
    std::cout << "Info_ww: " << Info_ww << std::endl;
    std::cout << "J_ww_cc: " << J_ww_cc << std::endl;
    std::cout << "Info_cc: " << Info_cc << std::endl;
#endif

    // Make sure the infor matrix is symmetric
    for(int i = 0; i < 6; i++)
        for(int j = 0; j < i; j++)
            Info_cc(i,j) = Info_cc(j,i);

    EdgeSE3ExpmapPrior* planeConstraint = new EdgeSE3ExpmapPrior();
    planeConstraint->setInformation(Info_cc);
#ifdef USE_EULER
    planeConstraint->setMeasurement(toSE3Quat(Tcw));
    if(true)
    {
        std::cout << "Info_cc: " << Info_cc << std::endl;
        std::cout << "measurement toSE3Quat(Tcw): " << toSE3Quat(Tcw) << std::endl;
    }
#else
    planeConstraint->setMeasurement(Tcw);
#endif
    planeConstraint->vertices()[0] = opt.vertex(vId);
    opt.addEdge(planeConstraint);
    return planeConstraint;
}


EdgeSE3ExpmapPrior*
addPlaneMotionSE3Expmap2(g2o::SparseOptimizer &opt, const g2o::SE3Quat &pose, int vId, const cv::Mat &extPara, bool bDebug) 
{
#ifdef USE_EULER
    const cv::Mat bTc = extPara;
    const cv::Mat cTb = bTc.inv();

    cv::Mat Tcw = toCvMat(pose);
    cv::Mat Tbw = bTc * Tcw;
    g2o::Vector3 euler = g2o::internal::toEuler( toMatrix3d(Tbw.rowRange(0,3).colRange(0,3)) );
    float yaw = euler(2);

    // Fix pitch and raw to zero, only yaw remains
    cv::Mat Rbw = (cv::Mat_<float>(3,3) <<
                   cos(yaw), -sin(yaw), 0,
                   sin(yaw),  cos(yaw), 0,
                   0,         0,        1);
    Rbw.copyTo(Tbw.rowRange(0,3).colRange(0,3));
    Tbw.at<float>(2,3) = 0; // Fix the height to zero

    Tcw = cTb * Tbw;
    //! Vector order: [rot, trans]
    Matrix6d Info_bw = Matrix6d::Zero();
    Info_bw(0,0) = 1e6;     // Rx
    Info_bw(1,1) = 1e6;     // Ry
    Info_bw(2,2) = 1e-4;    // Rz
    Info_bw(3,3) = 1e-4;    // tx
    Info_bw(4,4) = 1e-4;    // ty
    Info_bw(5,5) = 1;       // tz
    Matrix6d J_bb_cc = toSE3Quat(bTc).adj();
    Matrix6d Info_cw = J_bb_cc.transpose() * Info_bw * J_bb_cc;
#else

    g2o::SE3Quat Tbc = toSE3Quat(extPara);
    g2o::SE3Quat Tbw = Tbc * pose;

    Eigen::AngleAxisd AngleAxis_bw(Tbw.rotation());
    Eigen::Vector3d Log_Rbw = AngleAxis_bw.angle() * AngleAxis_bw.axis();
    AngleAxis_bw = Eigen::AngleAxisd(Log_Rbw[2], Eigen::Vector3d::UnitZ());
    Tbw.setRotation(Eigen::Quaterniond(AngleAxis_bw));

    Eigen::Vector3d xyz_bw = Tbw.translation();
    xyz_bw[2] = 0;
    Tbw.setTranslation(xyz_bw);

    g2o::SE3Quat Tcw = Tbc.inverse() * Tbw;

    //! Vector order: [rot, trans]
    Matrix6d Info_bw = Matrix6d::Zero();
    Info_bw(0,0) = 1e6;
    Info_bw(1,1) = 1e6;
    Info_bw(2,2) = 1e-4;
    Info_bw(3,3) = 1e-4;
    Info_bw(4,4) = 1e-4;
    Info_bw(5,5) = 1;
    Matrix6d J_bb_cc = Tbc.adj();
    Matrix6d Info_cw = J_bb_cc.transpose() * Info_bw * J_bb_cc;
#endif

    // Make sure the infor matrix is symmetric
    for(int i = 0; i < 6; i++)
        for(int j = 0; j < i; j++)
            Info_cw(i,j) = Info_cw(j,i);

    EdgeSE3ExpmapPrior* planeConstraint = new EdgeSE3ExpmapPrior();
    planeConstraint->setInformation(Info_cw);
#ifdef USE_EULER
    planeConstraint->setMeasurement(toSE3Quat(Tcw));
#else
    planeConstraint->setMeasurement(Tcw);
#endif
    planeConstraint->vertices()[0] = opt.vertex(vId);
    opt.addEdge(planeConstraint);

    return planeConstraint;

}