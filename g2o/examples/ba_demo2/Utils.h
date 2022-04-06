
#ifndef UTILS_H_
#define UTILS_H_


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

using clk = std::chrono::high_resolution_clock;
using time_point = std::chrono::time_point<clk>;
using dur_double = std::chrono::duration<double>;
using std::chrono::duration_cast;

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
  for(int i = 0; i < 4; i++)
    for(int j = 0; j < 4; j++)
      matPose.at<double>(i,j) = pose(i, j);

  if(bDebug)
  {
    cout << "pose: " << pose << endl;
    cout << "matPose: " << matPose << endl;
  }
}


#endif // UTILS_H_