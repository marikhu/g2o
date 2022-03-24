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

#include <iostream>
#include <unordered_set>

#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/structure_only/structure_only_solver.h"
#include "g2o/stuff/sampler.h"
#include "g2o/types/sba/types_six_dof_expmap.h"

#include "DataReader.h"

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
using namespace std;

class Sample {
public:
  static int uniform(int from, int to) {
    return static_cast<int>(g2o::Sampler::uniformRand(from, to));
  }
};

// https://gist.github.com/shubh-agrawal/76754b9bfb0f4143819dbd146d15d4c8
void getQuaternion(Mat R, double Q[])
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

    cout << "Q: " << Q[0] << " " << Q[1] << " " << Q[2] << " " << Q[3] << endl;
}

void getPoseFromTrfMat(Mat matTrf, g2o::SE3Quat &pose, bool bDebug = false )
{
  Mat matR(3,3,CV_64FC1);
  Mat matt(3,1,CV_64FC1);
  matTrf(Rect(0,0,3,3)).copyTo(matR);
  matTrf(Rect(3,0,1,3)).copyTo(matt);
  cout << "matTrf: " << matTrf << endl << "matR: " << matR << endl << "matt: " << matt << endl;
  // Mat matEulerAngles;
  // cv::Rodrigues(matR, matEulerAngles);

  double Q[4];
  getQuaternion(matR, Q);
  Eigen::Quaterniond q(Q[3],Q[0],Q[1],Q[2]); // w, x, y, z
  cout << "q: " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
  Vector3d trans(matt.at<double>(0), matt.at<double>(1), matt.at<double>(2));
  cout << "trans: " << trans << endl;
  pose = g2o::SE3Quat(q, trans);
  cout << "pose from getPoseFromTrfMat(): " << pose << endl;
}

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

  /////////////////////////////////////////////////////////////////////////////                                                                  
  
  // Load data
  DataReader *pDataReader = new DataReader();
  vector<vector<tsPolygon>> vvPolygonsInFlow;
  bool bDebug = true;
  pDataReader->setFileGeneratedPts2d3dInFlow(YML_GENERATED_PTS2D3D_FLOW, vvPolygonsInFlow, bDebug);
  vector<Vector3d> true_points;
  int iNumPolygonsToConsider = 3;
  pDataReader->getTruePoints(vvPolygonsInFlow[0], iNumPolygonsToConsider, true_points, bDebug);
  cout << "# true_points: " << true_points.size() << endl;

  Mat matP, matHiToG, matHgToI, matK, matD;
  pDataReader->setFileExtrinsics(YML_EXTRNSICS, matP, matHgToI, matHiToG, bDebug);
  pDataReader->setFileIntrinsics(YML_INTRINSICS, matK, matD, bDebug);

  /////////////////////////////////////////////////////////////////////////////                                                                  

  // // Config
  // int iNumPts = 30;
  int iNumPoses = 5;

  // cout << "true_points:" << endl;
  // for (size_t i = 0; i < iNumPts; ++i) {
  //   true_points.push_back(
  //       Vector3d((g2o::Sampler::uniformRand(0., 1.) - 0.5) * 3,
  //               g2o::Sampler::uniformRand(0., 1.) - 0.5,
  //               g2o::Sampler::uniformRand(0., 1.) + 3));
  //   cout << i << ": " << true_points[i][0] << ", " << true_points[i][1] << ", " << true_points[i][2] << endl;
  //   //cout << true_points[i] << endl;
  // }

  // Camera matrix K
  // double focal_length = 1000.;
  // Vector2d principal_point(320., 240.);

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
  getPoseFromTrfMat(matRt, pose0);

  // Add pose to vertex
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
  Mat newTrfForPose = Mat::zeros(4,4,CV_64FC1);
  matRt.copyTo(newTrfForPose(Rect(0,0,4,3)));
  newTrfForPose.at<double>(3,3) = 1;
  cout << "newTrfForPose: " << newTrfForPose << endl;

  for(int iTrfIdx = 1; iTrfIdx < (int)vMatTrfs.size(); iTrfIdx++)
  {
      Mat matTrf = vMatTrfs[iTrfIdx];
      // Now, we need Trf of World wrt Cam
      Mat matTrfInv = matTrf.inv();
      cout << "matTrf: " << matTrf << endl;
      newTrfForPose = newTrfForPose * matTrf;
      cout << "newTrfForPose: " << newTrfForPose << endl;
      g2o::SE3Quat pose;
      getPoseFromTrfMat(newTrfForPose, pose);
      g2o::VertexSE3Expmap* v_se3 = new g2o::VertexSE3Expmap();
      v_se3->setId(vertex_id);
      if(iTrfIdx < 2)
      {
        v_se3->setFixed(true); //-- In example, pose0 and pose1 set as fixed
      }
      v_se3->setEstimate(pose);
      optimizer.addVertex(v_se3);
      true_poses.push_back(pose);
      vertex_id++;
  }

  cout << "# true poses: " << true_poses.size() << endl;

  // int vertex_id = 0;
  // for (size_t i = 0; i < iNumPoses; ++i) {
  //   Vector3d trans(i * 0.04 - 1., 0, 0);
  //   //cout << endl << "trans " << i << ": " << endl << trans << endl;

  //   Eigen::Quaterniond q;
  //   q.setIdentity();
  //   cout << q.x() << " " << q.y() << " " << q.z() << endl;
  //   g2o::SE3Quat pose(q, trans);
  //   cout << "pose " << i << ": " << endl << pose << endl;
  //   g2o::VertexSE3Expmap* v_se3 = new g2o::VertexSE3Expmap();
  //   v_se3->setId(vertex_id);
  //   if (i < 2) {
  //     v_se3->setFixed(true);
  //   }
  //   v_se3->setEstimate(pose);
  //   optimizer.addVertex(v_se3);
  //   true_poses.push_back(pose);
  //   vertex_id++;
  // }
  int point_id = vertex_id;
  int point_num = 0;
  double sum_diff2 = 0;

  cout << endl;
  unordered_map<int, int> pointid_2_trueid;
  unordered_set<int> inliers;

  for (size_t i = 0; i < true_points.size(); ++i) {
    g2o::VertexPointXYZ* v_p = new g2o::VertexPointXYZ();
    v_p->setId(point_id);
    v_p->setMarginalized(true);
    v_p->setEstimate(true_points.at(i) +
                    Vector3d(g2o::Sampler::gaussRand(0., 1),
                              g2o::Sampler::gaussRand(0., 1),
                              g2o::Sampler::gaussRand(0., 1)));
    int num_obs = 0;
    for (size_t j = 0; j < true_poses.size(); ++j) {
      Vector2d z = cam_params->cam_map(true_poses.at(j).map(true_points.at(i)));
      if(i<2 && j <3)
      {
          cout << "projection is " << endl 
                << z << endl 
                << " for 3d point (index " << i << ") " << endl 
                <<  true_points.at(i) << endl 
                << " going through trf (index " << j << ") " << endl 
                << true_poses.at(j) << endl;
      }
      if (z[0] >= 0 && z[1] >= 0 && z[0] < 1920 && z[1] < 1080) {
        ++num_obs;
      }
    }
    if (num_obs >= 2) {
      optimizer.addVertex(v_p);
      bool inlier = true;
      for (size_t j = 0; j < true_poses.size(); ++j) {
        Vector2d z =
            cam_params->cam_map(true_poses.at(j).map(true_points.at(i)));

        if (z[0] >= 0 && z[1] >= 0 && z[0] < 1920 && z[1] < 1080) {
          double sam = g2o::Sampler::uniformRand(0., 1.);
          if (sam < OUTLIER_RATIO) {
            z = Vector2d(Sample::uniform(0, 1920), Sample::uniform(0, 1080));
            inlier = false;
          }
          z += Vector2d(g2o::Sampler::gaussRand(0., PIXEL_NOISE),
                        g2o::Sampler::gaussRand(0., PIXEL_NOISE));
          g2o::EdgeProjectXYZ2UV* e = new g2o::EdgeProjectXYZ2UV();
          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(v_p));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertices().find(j)->second));
          e->setMeasurement(z);
          e->information() = Matrix2d::Identity();
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
        Vector3d diff = v_p->estimate() - true_points[i];

        sum_diff2 += diff.dot(diff);
      }
      pointid_2_trueid.insert(make_pair(point_id, i));
      ++point_id;
      ++point_num;
    }
  }

  // Display pointid_2_true_id
  int iMapIdx = 0;
  for (unordered_map<int, int>::iterator it = pointid_2_trueid.begin();
      it != pointid_2_trueid.end(); ++it) {
    cout << "iMapIdx " << iMapIdx++ << " it->first " <<  it->first << ", it->second " << it->second << endl;   
  }


  cout << endl;
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
    structure_only_ba.calc(points, 10);
  }

  //////////////////////////////////////////////////////////////
  // optimizer.save("test.g2o");
  cout << endl;
  cout << "///////////////////////////////////" << endl;
  cout << "Performing full BA:" << endl;
  optimizer.optimize(10);
  cout << endl;
  cout << "Point error before optimisation (inliers only): "
      << sqrt(sum_diff2 / inliers.size()) << endl;
  point_num = 0;
  sum_diff2 = 0;
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
    Vector3d diff = v_p->estimate() - true_points[it->second];
    if (inliers.find(it->first) == inliers.end()) continue;
    sum_diff2 += diff.dot(diff);
    ++point_num;
  }
  cout << "Point error after optimisation (inliers only): "
      << sqrt(sum_diff2 / inliers.size()) << endl;
  cout << "///////////////////////////////////" << endl;
  cout << endl;
  //////////////////////////////////////////////////////////////

  for(int i = 0; i < iNumPoses; i++)
  {
    g2o::HyperGraph::VertexIDMap::iterator v_it =  optimizer.vertices().find(i);
    if (v_it == optimizer.vertices().end()) {
              cerr << "Vertex " << i << " not in graph!" << endl;
              exit(-1);
          } 

    g2o::VertexSE3Expmap* v_pose = dynamic_cast<g2o::VertexSE3Expmap*>(v_it->second);
    cout << v_pose->estimate() << endl;
  }
}
