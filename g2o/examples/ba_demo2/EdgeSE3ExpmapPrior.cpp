#include "EdgeSE3ExpmapPrior.h"

EdgeSE3ExpmapPrior::EdgeSE3ExpmapPrior() : g2o::BaseUnaryEdge<6, g2o::SE3Quat, g2o::VertexSE3Expmap>() {
    setMeasurement(g2o::SE3Quat());
    information().setIdentity();
}

void EdgeSE3ExpmapPrior::computeError(){
    g2o::VertexSE3Expmap *v = static_cast<g2o::VertexSE3Expmap*>(_vertices[0]);
    g2o::SE3Quat err = _measurement * v->estimate().inverse() ;
//    _error = _measurement.log() - v->estimate().log();
//    SE3Quat err = v->estimate().inverse() * _measurement;
//    SE3Quat err = _measurementInverse * v->estimate();
//    Eigen::AngleAxisd err_angleaxis(err.rotation());
//    _error.head<3>() = err_angleaxis.angle() * err_angleaxis.axis();
    _error = err.log();
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

    // ToDo:
    // Need to understand the frames 
    // How to enforce roll and pitch to be 0 and allow for little perturbations in tz or set to 0 as well
    // How to compute error.

    // MAYBE check on Huber!!!

#define USE_EULER

#ifdef USE_EULER
    const cv::Mat bTc = matT_CwrtW_init_fixed;  // T_WwrtC, pose0
    const cv::Mat cTb = bTc.inv(); // T_CwrtW, in order to work on T_WwrtW where constraints are to be enforced.

    if(bDebug)
    {
        std::cout << "bTc = matT_CwrtW_init_fixed:" << std::endl << matT_CwrtW_init_fixed << std::endl;
        std::cout << "cTb = bTc.inv():" << std::endl << cTb << std::endl;
    }

    cv::Mat Tcw = toCvMat(pose);    // T WwrtC pose_i
    std::cout << "Tcw = toCvMat(pose): " << Tcw << std::endl;
    
    std::cout << "bTc.type: " << bTc.type() << std::endl;
    std::cout << "Tcw.type: " << Tcw.type() << std::endl;
    cv::Mat Tbw = bTc * Tcw;
    std::cout << "Tbw.type: " << Tbw.type() << std::endl;
    std::cout << "Tbw = bTc * Tcw: " << Tbw << std::endl;

    Mat matR(3,3,CV_64FC1);
    Tbw(Rect(0,0,3,3)).copyTo(matR);
    // NOTE:  Tbw.rowRange(0,3).colRange(0,3) returns a matrix of type CV_32FC1
    // g2o::Vector3 euler = g2o::internal::toEuler(toMatrix3d(Tbw.rowRange(0,3).colRange(0,3)));
    // std::cout << "matR: " << matR << std::endl;
    // std::cout << "matR.type(): " << matR.type() << std::endl;
    // std::cout << "toMatrix3d(matR): " << toMatrix3d(matR) << std::endl;
    g2o::Vector3 euler = g2o::internal::toEuler(toMatrix3d(matR));
    std::cout << "euler: " << euler << std::endl;
    double yaw = euler(2);
    std::cout << "yaw: " << yaw << std::endl;

    // Fix pitch and raw to zero, only yaw remains
    cv::Mat Rbw = (cv::Mat_<double>(3,3) <<
                   cos(yaw), -sin(yaw), 0,
                   sin(yaw),  cos(yaw), 0,
                   0,         0,        1);
    std::cout << "Rbw: " << Rbw << std::endl;
    //Rbw.copyTo(Tbw.rowRange(0,3).colRange(0,3));
    Rbw.copyTo(Tbw(Rect(0,0,3,3)));
    //Tbw.at<double>(2,3) = 0; // Fix the height to zero
    std::cout << "Tbw: " << Tbw << std::endl;

    Tcw = cTb * Tbw;
    std::cout << "Tcw = cTb * Tbw: " << Tcw << std::endl;

    //! Vector order: [rot, trans]
    Matrix6d Info_bw = Matrix6d::Zero();
    Info_bw(0,0) = 1e6;
    Info_bw(1,1) = 1e6;
    Info_bw(2,2) = 1e-4;
    Info_bw(3,3) = 1e-4;
    Info_bw(4,4) = 1e-4;
    Info_bw(5,5) = 1;
    std::cout << "Info_bw: " << Info_bw << std::endl;
    Matrix6d J_bb_cc = toSE3Quat(bTc).adj();
    // Transfer of covariance thing
    Matrix6d Info_cw = J_bb_cc.transpose() * Info_bw * J_bb_cc;
#else

    g2o::SE3Quat Tbc = toSE3Quat(matT_CwrtW_init_fixed.inv());
    g2o::SE3Quat Tbw = Tbc * pose;

    Eigen::AngleAxisd AngleAxis_bw(Tbw.rotation());
    Eigen::Vector3d Log_Rbw = AngleAxis_bw.angle() * AngleAxis_bw.axis();
    AngleAxis_bw = Eigen::AngleAxisd(Log_Rbw[2], Eigen::Vector3d::UnitZ());
    Tbw.setRotation(Eigen::Quaterniond(AngleAxis_bw));

    Eigen::Vector3d xyz_bw = Tbw.translation();
    xyz_bw[2] = 0;  // tz
    Tbw.setTranslation(xyz_bw);

    g2o::SE3Quat Tcw = Tbc.inverse() * Tbw;

    //! Vector order: [rot, trans]
    Matrix6d Info_bw = Matrix6d::Zero();
    // Information is Hessian matrix, i.e. inverse of covariance matrix
    Info_bw(0,0) = 1e6;     // Rx
    Info_bw(1,1) = 1e6;     // Ry
    Info_bw(2,2) = 1e-4;    // Rz    
    Info_bw(3,3) = 1e-4;    // tx    
    Info_bw(4,4) = 1e-4;    // ty
    Info_bw(5,5) = 1;       // tz
    Matrix6d J_bb_cc = Tbc.adj();
    Matrix6d Info_cw = J_bb_cc.transpose() * Info_bw * J_bb_cc;
    std::cout << "Info_bw: " << Info_bw << std::endl;
    std::cout << "J_bb_cc: " << J_bb_cc << std::endl;
    std::cout << "Info_cw: " << Info_cw << std::endl;
#endif

    // Make sure the infor matrix is symmetric
    for(int i = 0; i < 6; i++)
        for(int j = 0; j < i; j++)
            Info_cw(i,j) = Info_cw(j,i);
    std::cout << "Info_cw: " << Info_cw << std::endl;

    EdgeSE3ExpmapPrior* planeConstraint = new EdgeSE3ExpmapPrior();
    planeConstraint->setInformation(Info_cw);
#ifdef USE_EULER
    planeConstraint->setMeasurement(toSE3Quat(Tcw));
    std::cout << "measurement toSE3Quat(Tcw): " << toSE3Quat(Tcw) << std::endl;
#else
    planeConstraint->setMeasurement(Tcw);
#endif
    planeConstraint->vertices()[0] = opt.vertex(vId);
    opt.addEdge(planeConstraint);
    return planeConstraint;
}
