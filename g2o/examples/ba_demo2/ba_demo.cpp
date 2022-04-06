// g2o - General Graph Optimization
// Copyright (C) 2011 H. Strasdat
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <stdint.h>
#include <signal.h>

#include <iostream>
#include <unordered_set>
#include <thread>
#include <random>
using namespace std;

#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/structure_only/structure_only_solver.h"
#include "g2o/stuff/sampler.h"
#include "g2o/types/sba/types_six_dof_expmap.h"

#include <cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
using namespace cv;

#if defined G2O_HAVE_CHOLMOD
  G2O_USE_OPTIMIZATION_LIBRARY(cholmod);
#else
  G2O_USE_OPTIMIZATION_LIBRARY(eigen);
#endif
G2O_USE_OPTIMIZATION_LIBRARY(dense);
using namespace Eigen;

using clk = std::chrono::high_resolution_clock;
using time_point = std::chrono::time_point<clk>;
using dur_double = std::chrono::duration<double>;
using std::chrono::duration_cast;

#include "DataReader.h"
#include "EdgeSE3ExpmapPrior.h"

/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////

class cTimer
{
public:
    cTimer(const std::string &cmd) : _cmd{cmd}, _start{clk::now()} {};

    double time_ns()
    {
        auto duration = clk::now() - _start;
        auto elapsed_s = duration_cast<dur_double>(duration).count();
        return elapsed_s * 1000 * 1000 * 1000;
    }

    ~cTimer(){};

private:
    std::string _cmd;
    time_point _start;
};

int intRand(const int &min, const int &max)
{
    static thread_local std::mt19937 generator;
    std::uniform_int_distribution<int> distribution(min, max);
    return distribution(generator);
}

class Sample {
public:
  static int uniform(int from, int to) {
    return static_cast<int>(g2o::Sampler::uniformRand(from, to));
  }
};

// https://gist.github.com/shubh-agrawal/76754b9bfb0f4143819dbd146d15d4c8
void getQuaternion(Mat R, double Q[], bool bDebug = false)
{
    double trace = R.at<double>(0,0) + R.at<double>(1,1) + R.at<double>(2,2);
 
    if (trace > 0.0) 
    {
        double s = sqrt(trace + 1.0);
        Q[3] = (s * 0.5);
        s = 0.5 / s;
        Q[0] = ((R.at<double>(2,1) - R.at<double>(1,2)) * s);
        Q[1] = ((R.at<double>(0,2) - R.at<double>(2,0)) * s);
        Q[2] = ((R.at<double>(1,0) - R.at<double>(0,1)) * s);
    } 
    
    else 
    {
        int i = R.at<double>(0,0) < R.at<double>(1,1) ? (R.at<double>(1,1) < R.at<double>(2,2) ? 2 : 1) : (R.at<double>(0,0) < R.at<double>(2,2) ? 2 : 0); 
        int j = (i + 1) % 3;  
        int k = (i + 2) % 3;

        double s = sqrt(R.at<double>(i, i) - R.at<double>(j,j) - R.at<double>(k,k) + 1.0);
        Q[i] = s * 0.5;
        s = 0.5 / s;

        Q[3] = (R.at<double>(k,j) - R.at<double>(j,k)) * s;
        Q[j] = (R.at<double>(j,i) + R.at<double>(i,j)) * s;
        Q[k] = (R.at<double>(k,i) + R.at<double>(i,k)) * s;
    }

    Mat matEulerAngles;
    cv::Rodrigues(R, matEulerAngles);
    if(bDebug)
    {
      cout << "R: " << R << endl;
      cout << "matEulerAngles: " << matEulerAngles << endl;
      cout << "Q: " << Q[0] << " " << Q[1] << " " << Q[2] << " " << Q[3] << endl;
    }
}

void getPoseFromTrfMat(Mat matTrf, g2o::SE3Quat &pose, bool bDebug = false)
{
  Mat matR(3,3,CV_64FC1);
  Mat matt(3,1,CV_64FC1);
  matTrf(Rect(0,0,3,3)).copyTo(matR);
  matTrf(Rect(3,0,1,3)).copyTo(matt);
  // Mat matEulerAngles;
  // cv::Rodrigues(matR, matEulerAngles);

  double Q[4];
  getQuaternion(matR, Q);
  Eigen::Quaterniond q(Q[3],Q[0],Q[1],Q[2]); // w, x, y, z
  
  Vector3d trans(matt.at<double>(0), matt.at<double>(1), matt.at<double>(2));
  pose = g2o::SE3Quat(q, trans);

  if(bDebug)
  {
    cout << "matTrf: " <<    matTrf << endl << "matR: " << matR << endl << "matt: " << matt << endl;
    cout << "q: " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
    cout << "trans: " << trans << endl;
    cout << "pose from getPoseFromTrfMat(): " << pose << endl;
  }
}

void getMat(Eigen::Matrix4d pose, Mat &matPose, bool bDebug = false)
{
  for(int i =0; i < 4; i++)
    for(int j = 0; j < 4; j++)
      matPose.at<double>(i,j) = pose(i, j);

  if(bDebug)
  {
    cout << "pose: " << pose << endl;
    cout << "matPose: " << matPose << endl;
  }
}


//////////////////
// main
//////////////////

int main(int argc, const char* argv[]) {
  if (argc < 2) {
    cout << endl;
    cout << "Please type: " << endl;
    cout << "ba_demo [PIXEL_NOISE] [OUTLIER RATIO] [ROBUST_KERNEL] "
            "[STRUCTURE_ONLY] [DENSE]"
        << endl;
    cout << endl;
    cout << "PIXEL_NOISE: noise in image space (E.g.: 1)" << endl;
    cout << "OUTLIER_RATIO: probability of spuroius observation  (default: 0.0)"
        << endl;
    cout << "ROBUST_KERNEL: use robust kernel (0 or 1; default: 0==false)"
        << endl;
    cout << "STRUCTURE_ONLY: performe structure-only BA to get better point "
            "initializations (0 or 1; default: 0==false)"
        << endl;
    cout << "DENSE: Use dense solver (0 or 1; default: 0==false)" << endl;
    cout << endl;
    cout << "Note, if OUTLIER_RATIO is above 0, ROBUST_KERNEL should be set to "
            "1==true."
        << endl;
    cout << endl;
    exit(0);
  }

  double PIXEL_NOISE = atof(argv[1]);
  double OUTLIER_RATIO = 0.0;

  if (argc > 2) {
    OUTLIER_RATIO = atof(argv[2]);
  }

  bool ROBUST_KERNEL = false;
  if (argc > 3) {
    ROBUST_KERNEL = atoi(argv[3]) != 0;
  }
  bool STRUCTURE_ONLY = false;
  if (argc > 4) {
    STRUCTURE_ONLY = atoi(argv[4]) != 0;
  }

  bool DENSE = false;
  if (argc > 5) {
        DENSE = atoi(argv[5]) != 0;
  }

  cout << "PIXEL_NOISE: " << PIXEL_NOISE << endl;
  cout << "OUTLIER_RATIO: " << OUTLIER_RATIO << endl;
  cout << "ROBUST_KERNEL: " << ROBUST_KERNEL << endl;
  cout << "STRUCTURE_ONLY: " << STRUCTURE_ONLY << endl;
  cout << "DENSE: " << DENSE << endl;

  g2o::SparseOptimizer optimizer;
  optimizer.setVerbose(false);
  string solverName = "lm_fix6_3";
  if (DENSE) {
    solverName = "lm_dense6_3";
  } else {
#ifdef G2O_HAVE_CHOLMOD
    solverName = "lm_fix6_3_cholmod";
#else
    solverName = "lm_fix6_3";
#endif
  }

  g2o::OptimizationAlgorithmProperty solverProperty;
  optimizer.setAlgorithm(
      g2o::OptimizationAlgorithmFactory::instance()->construct(solverName,
                                                              solverProperty));
  optimizer.setVerbose(true);                                                              

  /////////////////////////////////////////////////////////////////////////////                                                                  
  
  /////////////
  // Config
  /////////////
  bool bDebug = false;
  int iNumPolygonsToConsider = 9; // If 0, all polygons are considered
  int iNumIterations = 10;
  int iImgWidth = 3000;
  int iImgHeight = 2000;
  float fGaussNoiseStdDev = 3.0; //3.0; //3.0f;
  bool bSetObservationsExplicitly = true;
  bool bInitTrfWtoWAsUnknown = true;

  // Load data
  DataReader *pDataReader = new DataReader();
  vector<vector<tsPolygon>> vvPolygonsInFlow;
  pDataReader->setFileGeneratedPts2d3dInFlow(YML_GENERATED_PTS2D3D_FLOW, vvPolygonsInFlow, bDebug);

  // Get 3D points for the first polygon, considered as the true points
  vector<Vector3d> true_points;
  if(iNumPolygonsToConsider == 0) iNumPolygonsToConsider = vvPolygonsInFlow[0].size();
  pDataReader->getTruePoints(vvPolygonsInFlow[0][0], true_points, bDebug);
  cout << "# true_points: " << true_points.size() << endl;

  // Get 2D observations 
  vector<Vector2d> v2dObservations;
  pDataReader->getObservations(vvPolygonsInFlow[0], iNumPolygonsToConsider, v2dObservations, bDebug);
  cout << "# observations: " << v2dObservations.size() << endl;
  if(bDebug)
  {
    for(int i = 0; i < (int)v2dObservations.size(); i++)
      cout << "i: " << i << " -- (" << v2dObservations[i].x() << ", " << v2dObservations[i].y() << ")" << endl;
  }

  Mat matP, matHiToG, matHgToI, matK, matD;
  pDataReader->setFileExtrinsics(YML_EXTRNSICS, matP, matHgToI, matHiToG, bDebug);
  pDataReader->setFileIntrinsics(YML_INTRINSICS, matK, matD, bDebug);

  // Initialize 3D points
  // Currently, true_points are added to the vertices with gaussian random noise of mean = 0, std_dev = 1 (in world frame, meters)
  // ToDo:
  //  - Initialize Az = 0 for the first point, Compute Ax, Ay
  //  - Initialize Xz = Az for all other points, Compute Xx, Xy
  // We have observations on the image
  //  - Given point on the image, point on normalized image plane in camera coordinates and point on the ground, we get a ray
  //  - Given height of a point on the ray, we can locate 3D point on the ray



  /////////////////////////////////////////////////////////////////////////////                                                                  

  double focal_length = matK.at<double>(0,0);
  double cx = matK.at<double>(0,2);
  double cy = matK.at<double>(1,2);
  Vector2d principal_point(cx, cy);
  cout <<"focal_length: " << focal_length << ", cx: " << cx << ", cy: " << cy << endl;

  g2o::CameraParameters* cam_params =
      new g2o::CameraParameters(focal_length, principal_point, 0.);
  cam_params->setId(0);

  if (!optimizer.addParameter(cam_params)) {
    assert(false);
  }

  // Get the camera pose ^C T _W -- Transformation of the World wrt Camera
  Mat matKinv = matK.inv();
  Mat matRt = matKinv * matP;   // Trf of World wrt Cam
  g2o::SE3Quat pose0;
  getPoseFromTrfMat(matRt, pose0, bDebug);

  // Add first pose to vertex -- Trf of World wrt Camera, obtained from P = K*[R|t]
  vector<g2o::SE3Quat, aligned_allocator<g2o::SE3Quat> > true_poses;
  int vertex_id = 0;
  g2o::VertexSE3Expmap* v_se3 = new g2o::VertexSE3Expmap();
  v_se3->setId(vertex_id);
  v_se3->setFixed(true);
  v_se3->setEstimate(pose0);
  optimizer.addVertex(v_se3);
  true_poses.push_back(pose0);
  vertex_id++;
  
  vector<Mat> vMatTrfs;
  pDataReader->getTrfs(vvPolygonsInFlow[0], iNumPolygonsToConsider, vMatTrfs, bDebug);
  // P0_C = T_WwrtC * P1_W                    -- pose for P1_W
  // P2_W = T_WwrtW * P1_W
  // P1_W = (T_WwrtW)^-1 * P2_W
  // P0_C = T_wrtC * (T_WwrtW)^-1 * P2_W      -- pose for P2_W
  Mat matPose_new = Mat::zeros(4,4,CV_64FC1);
  matRt.copyTo(matPose_new(Rect(0,0,4,3)));
  matPose_new.at<double>(3,3) = 1;
  cout << "pose0 -- Rt from T_WwrtC: " << matPose_new << endl;

  Mat matPose_old = matPose_new.clone();
  Mat matT_WwrtW_cur;
  for(int iTrfIdx = 1; iTrfIdx < (int)vMatTrfs.size(); iTrfIdx++)
  {
      Mat matTrf = vMatTrfs[iTrfIdx];
      // Now, we need Trf of new World frame wrt Cam frame
      if(bInitTrfWtoWAsUnknown)
      {
        // Here, initializing tx, ty, Rz to 0, 
        // considering identity for rotation matrix and zero vector for translation matrix
        matPose_new = matPose_new;
      }
      else
        matPose_new = matPose_new * matTrf;

    //  matPose_new.at<double>(0,0) += g2o::Sampler::gaussRand(0., 0.1);
    //  matPose_new.at<double>(1,1) += g2o::Sampler::gaussRand(0., 0.1);
    //  matPose_new.at<double>(2,2) += g2o::Sampler::gaussRand(0., 0.1);
    //  matPose_new.at<double>(0,3) += g2o::Sampler::gaussRand(0., 0.2);
    //  matPose_new.at<double>(1,3) += g2o::Sampler::gaussRand(0., 0.2);
    //  matPose_new.at<double>(3,3) += g2o::Sampler::gaussRand(0., 0.2);

      cout << "iTrfIdx: " << iTrfIdx << endl << "matTrf: " << endl << matTrf << endl;

      // Recovering matTrf
      matT_WwrtW_cur = matPose_old.inv() * matPose_new;
      matPose_new.copyTo(matPose_old);

      cout << iTrfIdx << ": matT_WwrtW_cur: " << matT_WwrtW_cur << endl;

      if(bDebug)
      {
        cout << "matTrf: " << endl << matTrf << endl;
        cout << "matPose_new: " << endl << matPose_new << endl;
      }
      g2o::SE3Quat pose;
      getPoseFromTrfMat(matPose_new, pose, bDebug);
      g2o::VertexSE3Expmap* v_se3 = new g2o::VertexSE3Expmap();
      v_se3->setId(vertex_id);
      if(iTrfIdx < 1)
      {
        v_se3->setFixed(true); //-- In example, pose0 and pose1 set as fixed. In our case only Trf of W wrt C is fixed.
      }
      v_se3->setEstimate(pose);
      optimizer.addVertex(v_se3);
      true_poses.push_back(pose);
      vertex_id++;
  }

  cout << "# true poses: " << true_poses.size() << endl;
 
  int point_id = vertex_id;
  int point_num = 0;
  double sum_diff2_W = 0;
  double sum_diff2_I_gt = 0;

  cout << endl;
  unordered_map<int, int> pointid_2_trueid;
  unordered_set<int> inliers;

  int iCount = 0;
  int iNumPtsPerPolygon = (int)true_points.size();
  for (size_t i = 0; i < iNumPtsPerPolygon; ++i) {
    g2o::VertexPointXYZ* v_p = new g2o::VertexPointXYZ();
    v_p->setId(point_id);
    v_p->setMarginalized(true);
    double dRandX = g2o::Sampler::gaussRand(0., fGaussNoiseStdDev);
    double dRandY = g2o::Sampler::gaussRand(0., fGaussNoiseStdDev);
    double dRandZ = g2o::Sampler::gaussRand(0., fGaussNoiseStdDev);
    v_p->setEstimate(true_points.at(i) + Vector3d(dRandX, dRandY, dRandZ));
    Vector3d ptW = v_p->estimate();

    // Checking for inliers/outliers
    int num_obs = 0;
    for (size_t j = 0; j < true_poses.size(); ++j) {
      Vector2d z;
      if(bSetObservationsExplicitly)
      {
        int iIdx = (i % iNumPtsPerPolygon)+ iNumPtsPerPolygon * j;
        //cout << "iIdx: " << iIdx << " for i " << i  << " and j " << j << endl;
        z = v2dObservations[iIdx];
      }
      else 
        z = cam_params->cam_map(true_poses.at(j).map(true_points.at(i)));

      if(bDebug)
      {
        Vector2d z2 = cam_params->cam_map(true_poses.at(j).map(true_points.at(i)));
        cout << iCount++ << " z: " << z.x() << ", " << z.y()  
              <<  " z2: " << z2.x() << ", " << z2.y() << " for i " << i  << " and j " << j << endl;;
      }
      
      if(i<2 && j <3)
      {
          if(bDebug) cout << "projection is " << endl 
                << z << endl 
                << " for 3d point (index " << i << ") " << endl 
                <<  true_points.at(i) << endl 
                << " going through trf (index " << j << ") " << endl 
                << true_poses.at(j) << endl;
      }
      if (z[0] >= 0 && z[1] >= 0 && z[0] < iImgWidth && z[1] < iImgHeight) {
        ++num_obs;
      }
    }

    // Considering those points which are inliers or those which are within the image dimensions
    cout << "num_obs for " << i << ": " << num_obs << endl;
    if (num_obs >= 2) {
      optimizer.addVertex(v_p);
      Vector2d ptI_gt = v2dObservations[i];
      bool inlier = true;
      for (size_t j = 0; j < true_poses.size(); ++j) {
        Vector2d z;
        if(bSetObservationsExplicitly)
        {
          int iIdx = (i % iNumPtsPerPolygon)+ iNumPtsPerPolygon * j;
          //cout << "iIdx: " << iIdx << " for i " << i  << " and j " << j << endl;
          z = v2dObservations[iIdx];
        }
        else 
          z = cam_params->cam_map(true_poses.at(j).map(true_points.at(i)));

        if (z[0] >= 0 && z[1] >= 0 && z[0] < iImgWidth && z[1] < iImgHeight) {
          double sam = g2o::Sampler::uniformRand(0., 1.);
          if (sam < OUTLIER_RATIO) {
            z = Vector2d(Sample::uniform(0, iImgWidth), Sample::uniform(0, iImgHeight));
            inlier = false;
          }
          z += Vector2d(g2o::Sampler::gaussRand(0., PIXEL_NOISE),
                        g2o::Sampler::gaussRand(0., PIXEL_NOISE));
          g2o::EdgeProjectXYZ2UV* e = new g2o::EdgeProjectXYZ2UV();
          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(v_p)); // 3D point
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertices().find(j)->second)); // 3D pose
          e->setMeasurement(z);
          e->information() = Matrix2d::Identity();  // Covariance matrix    
          if (ROBUST_KERNEL) {
            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
          }
          e->setParameterId(0, 0);
          optimizer.addEdge(e);
        }
      }

      if (inlier) {
        inliers.insert(point_id);
        Vector3d diffW = v_p->estimate() - true_points[i];
        sum_diff2_W += diffW.dot(diffW);

        // Get projection of all ptW for the poses
        for (size_t j = 0; j < true_poses.size(); ++j) 
        {          
          int iObsIdx = i + j * iNumPtsPerPolygon;
          Vector2d ptI_gt = v2dObservations[iObsIdx];
          // Get the 3D point
          g2o::HyperGraph::VertexIDMap::iterator v_it =  optimizer.vertices().find(j);
          if (v_it == optimizer.vertices().end()) {
                cerr << "Vertex " << j << " not in graph!" << endl;
                exit(-1);
          } 
          g2o::VertexSE3Expmap* v_pose = dynamic_cast<g2o::VertexSE3Expmap*>(v_it->second);

          Vector2d ptI_reproj = cam_params->cam_map(v_pose->estimate().map(v_p->estimate()));
          Vector2d diffI_gt = ptI_gt - ptI_reproj;
          sum_diff2_I_gt = diffI_gt.dot(diffI_gt);
          if(bDebug)
          {
            cout << endl << "point index i: " << i << endl;
            cout << "pose index j: " << j << endl;
            cout << "true_point: " << endl << true_points[i] << endl;
            cout << "v_p init: " << endl << v_p->estimate() << endl;
            cout << "diffW: " << diffW << endl;
            cout << "v_pose init: " << endl << v_pose->estimate() << endl;
            cout << "ptI_gt: " << endl << ptI_gt << endl;
            cout << "ptI_reproj: " << endl << ptI_reproj << endl;
            cout << "diffI_gt : " << diffI_gt << endl;
          }
        }
      }
      pointid_2_trueid.insert(make_pair(point_id, i));
      ++point_id;
      ++point_num;
    }
  }

  int iNumEdges = (int)optimizer.edges().size();
  cout << "# of edges: " << iNumEdges << endl;

  if(bDebug)
  {
    // Display pointid_2_true_id
    int iMapIdx = 0;
    for (unordered_map<int, int>::iterator it = pointid_2_trueid.begin();
        it != pointid_2_trueid.end(); ++it) {
      cout << "iMapIdx " << iMapIdx++ << " it->first " <<  it->first << ", it->second " << it->second << endl;   
    }
  }
  cout << endl;

  // Optimize
  optimizer.initializeOptimization();
  optimizer.setVerbose(true);

  if (STRUCTURE_ONLY) {
    g2o::StructureOnlySolver<3> structure_only_ba;
    cout << "Performing structure-only BA:" << endl;
    g2o::OptimizableGraph::VertexContainer points;
    for (g2o::OptimizableGraph::VertexIDMap::const_iterator it =
            optimizer.vertices().begin();
        it != optimizer.vertices().end(); ++it) {
      g2o::OptimizableGraph::Vertex* v =
          static_cast<g2o::OptimizableGraph::Vertex*>(it->second);
      if (v->dimension() == 3) points.push_back(v);
    }
    structure_only_ba.calc(points, iNumIterations);
  }

  //////////////////////////////////////////////////////////////
  optimizer.save("input_graph.g2o");
  cout << endl;
  cout << "///////////////////////////////////" << endl;
  cout << "Performing full BA:" << endl;

  // Time the optimization
  auto t = cTimer{__FUNCTION__};
  optimizer.optimize(iNumIterations, false);
  double dTimeNs = t.time_ns();
  //////////////////////////////////////////////////////////////

  if(true)
  {
    Mat matT_WwrtW_cur;
    Mat matPose_old;
    for(int i = 0; i < (int)true_poses.size(); i++)
    {
      g2o::HyperGraph::VertexIDMap::iterator v_it =  optimizer.vertices().find(i);
      if (v_it == optimizer.vertices().end()) 
      {
                cerr << "Vertex " << i << " not in graph!" << endl;
                exit(-1);
      }
      g2o::VertexSE3Expmap* v_pose = dynamic_cast<g2o::VertexSE3Expmap*>(v_it->second);
      
      Eigen::Isometry3d poseOpt = v_pose->estimate();
      if(bDebug)
      {
        cout << "translation: " << endl << v_pose->estimate().translation() << endl;
        cout << "toMinimalVector: " << v_pose->estimate().toMinimalVector() << endl;      
        cout << "Pose=" << endl << poseOpt.matrix() <<endl;
        for(int i = 0; i < 16; i ++)   // Traversal is column-wise
          cout << "Pose[" << i << "]=" << endl << poseOpt.matrix()(i) <<endl;
      }

      // ToDo: Extract T_W_wrtW to compare with GT in input YML file      
      Mat matPoseOpt(4,4,CV_64FC1);
      getMat(poseOpt.matrix(), matPoseOpt);
      if(i==0) matPose_old = matPoseOpt.clone();
      else
        matT_WwrtW_cur = matPose_old.inv() * matPoseOpt; 
        matPoseOpt.copyTo(matPose_old);
      if(matT_WwrtW_cur.data)
      {
        cout <<  "matT_WwrtW_cur " << i << endl << matT_WwrtW_cur << endl;
        // Obtain tx, ty, Rz, 
        double tx = matT_WwrtW_cur.at<double>(0,3);
        double ty = matT_WwrtW_cur.at<double>(1,3);
        double tz = matT_WwrtW_cur.at<double>(2,3);
        Mat matR(3,3,CV_64FC1);
        matT_WwrtW_cur(Rect(0,0,3,3)).copyTo(matR);
        Mat matR_Euler(1,3,CV_64FC1);
        cv::Rodrigues(matR, matR_Euler);
        cout << "matR_Euler: " << endl << matR_Euler << endl;
        matR_Euler = matR_Euler * 180 / CV_PI;
        cout << "matR_Euler (deg): " << endl << matR_Euler << endl;
        cout << "tx: " << tx << ", ty: " << ty << ", tz: " << tz << endl;
      }
    }
  }

  cout << endl;
  cout << "Point error before optimization (inliers only) 3D world points: "
      << sqrt(sum_diff2_W / inliers.size()) << endl;
  cout << "Point error before optimization (inliers only) 2D image points: "
      << sqrt(sum_diff2_I_gt / ((int)inliers.size() * iNumPolygonsToConsider)) << endl;

  point_num = 0;
  sum_diff2_W = 0;
  sum_diff2_I_gt = 0;
  double sum_diff2_I_meas = 0;

  for (unordered_map<int, int>::iterator it = pointid_2_trueid.begin();
      it != pointid_2_trueid.end(); ++it) {
    g2o::HyperGraph::VertexIDMap::iterator v_it =
        optimizer.vertices().find(it->first);
    if (v_it == optimizer.vertices().end()) {
      cerr << "Vertex " << it->first << " not in graph!" << endl;
      exit(-1);
    }
    g2o::VertexPointXYZ* v_p = dynamic_cast<g2o::VertexPointXYZ*>(v_it->second);
    if (v_p == 0) {
      cerr << "Vertex " << it->first << "is not a PointXYZ!" << endl;
      exit(-1);
    }
    Vector3d diffW = v_p->estimate() - true_points[it->second];    
    if (inliers.find(it->first) == inliers.end()) continue;
    sum_diff2_W += diffW.dot(diffW);

    // Get projection of all ptW for the poses
    for (size_t j = 0; j < true_poses.size(); ++j) 
    {         
      int iObsIdx = it->second + j * iNumPtsPerPolygon;
      Vector2d ptI_gt = v2dObservations[iObsIdx];    
      // Get the 3D point
      g2o::HyperGraph::VertexIDMap::iterator v_it =  optimizer.vertices().find(j);
      if (v_it == optimizer.vertices().end()) 
      {
            cerr << "Vertex " << j << " not in graph!" << endl;
            exit(-1);
      } 
      g2o::VertexSE3Expmap* v_pose = dynamic_cast<g2o::VertexSE3Expmap*>(v_it->second);
      Vector2d ptI_reproj = cam_params->cam_map(v_pose->estimate().map(v_p->estimate()));

      Vector2d ptI_meas;

      // Given vertex index and a pose index, we should be able to find the edge, and its measurement
      bool bEdgeFound = false;
      for (g2o::HyperGraph::EdgeSet::const_iterator it = optimizer.edges().begin(); 
        it != optimizer.edges().end(); ++it) 
      {
        g2o::EdgeProjectXYZ2UV* scanmatchEdge = dynamic_cast<g2o::EdgeProjectXYZ2UV*>(*it);
        if (! scanmatchEdge)
          continue;

        g2o::VertexPointXYZ* e_v_p = dynamic_cast<g2o::VertexPointXYZ*>(scanmatchEdge->vertices()[0]);
        g2o::VertexSE3Expmap* e_v_pose = dynamic_cast<g2o::VertexSE3Expmap*>(scanmatchEdge->vertices()[1]);
        if(e_v_p->id() == v_p->id() && e_v_pose->id() == v_pose->id())
        {
          bEdgeFound = true;
          ptI_meas = scanmatchEdge->measurement();
          if(bDebug)
          {
            cout << "We got a matching edge" << endl;
            cout << "ptI_meas: " << endl << ptI_meas << endl;
            cout << "ptI_gt: " << endl << ptI_gt << endl;
          }
          break;
        }
      }
      if(!bEdgeFound)
      {
        cerr << "Edge not found." << endl;
        exit(-1);
      }

      Vector2d diffI_gt = ptI_gt - ptI_reproj;
      sum_diff2_I_gt = diffI_gt.dot(diffI_gt);

      Vector2d diffI_meas = ptI_meas - ptI_reproj;
      sum_diff2_I_meas = diffI_meas.dot(diffI_meas);

      if(bDebug)
      {
        cout << endl << "iObsIdx: " << iObsIdx << endl;
        cout << "point index i: " << it->second << endl;
        cout << "pose index j: " << j << endl;
        cout << "true_point: " << endl << true_points[it->second] << endl;
        cout << "v_p opt: " << endl << v_p->estimate() << endl;
        cout << "diffW: " << diffW << endl;
        cout << "v_pose opt: " << endl << v_pose->estimate() << endl;
        cout << "ptI_gt: " << endl << ptI_gt << endl;
        cout << "ptI_reproj: " << endl << ptI_reproj << endl;
        cout << "diffI_gt: " << diffI_gt << endl;
        cout << "diffI_meas: " << diffI_meas << endl;
      }
    }
    ++point_num;
  }
  
  cout << endl <<  "Point error after optimization (inliers only) 3D world points: "
      << sqrt(sum_diff2_W / inliers.size()) << endl;
  cout << "Point error after optimization (inliers only) 2D image points (w.r.t ptI_gt): "
      << sqrt(sum_diff2_I_gt / ((int)inliers.size() * iNumPolygonsToConsider)) << endl;
  cout << "Point error after optimization (inliers only) 2D image points (w.r.t ptI_meas): "
      << sqrt(sum_diff2_I_meas / ((int)inliers.size() * iNumPolygonsToConsider)) << endl;
  cout << endl;

  cout << "# of inliers: " << (int)inliers.size() << endl;
  cout << "optimization time: " <<dTimeNs / 1e6 << " ms" << endl;

  if(false)
  {
    int k = 0;
    int iOccurrences = 0;
    for (g2o::HyperGraph::EdgeSet::const_iterator it = optimizer.edges().begin(); it != optimizer.edges().end(); ++it) {
      g2o::EdgeProjectXYZ2UV* scanmatchEdge = dynamic_cast<g2o::EdgeProjectXYZ2UV*>(*it);
      if (! scanmatchEdge)
        continue;

      g2o::VertexPointXYZ* v_p = dynamic_cast<g2o::VertexPointXYZ*>(scanmatchEdge->vertices()[0]);
      g2o::VertexSE3Expmap* v_pose = dynamic_cast<g2o::VertexSE3Expmap*>(scanmatchEdge->vertices()[1]);
      cout << k++ << " v_p id: " << v_p->id() << ", v_pose id: " << v_pose->id() << endl;
      cout << "scanmatchEdge->measurement: " << endl << scanmatchEdge->measurement() << endl;

      if(v_p->id()==9) iOccurrences++;
    }
    cout << "iOccurences: " << iOccurrences << endl;
  }
}
