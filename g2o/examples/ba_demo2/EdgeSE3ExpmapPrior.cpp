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
    R << cvT.at<float>(0,0), cvT.at<float>(0,1), cvT.at<float>(0,2),
            cvT.at<float>(1,0), cvT.at<float>(1,1), cvT.at<float>(1,2),
            cvT.at<float>(2,0), cvT.at<float>(2,1), cvT.at<float>(2,2);

    Eigen::Matrix<double,3,1> t(cvT.at<float>(0,3), cvT.at<float>(1,3), cvT.at<float>(2,3));

    return g2o::SE3Quat(R,t);
}

Matrix6d toMatrix6d(const cv::Mat &cvMat6d)
{
    Matrix6d m = Matrix6d::Zero();
    for(int i = 0; i < 6; i++)
        for(int j = 0; j < 6; j++)
            m(i,j) = cvMat6d.at<float>(i,j);
    return m;
}


EdgeSE3ExpmapPrior* addPlaneMotionSE3Expmap(
    g2o::SparseOptimizer &opt, const g2o::SE3Quat &pose, int vId, const cv::Mat &extPara)
{

//#define USE_EULER

#ifdef USE_EULER
    const cv::Mat bTc = extPara;
    const cv::Mat cTb = scv::inv(bTc);

    cv::Mat Tcw = toCvMat(pose);
    cv::Mat Tbw = bTc * Tcw;
    g2o::Vector3D euler = g2o::internal::toEuler( toMatrix3d(Tbw.rowRange(0,3).colRange(0,3)) );
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
    Info_bw(0,0) = Config::PLANEMOTION_XROT_INFO;
    Info_bw(1,1) = Config::PLANEMOTION_YROT_INFO;
    Info_bw(2,2) = 1e-4;
    Info_bw(3,3) = 1e-4;
    Info_bw(4,4) = 1e-4;
    Info_bw(5,5) = Config::PLANEMOTION_Z_INFO;
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
