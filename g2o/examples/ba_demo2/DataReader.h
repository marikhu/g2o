#ifndef DATAREADER_H_
#define DATAREADER_H_

#include <vector>
#include <iostream>
#include <string>
using namespace std;

#include <stdint.h>

#include <iostream>
#include <unordered_set>

#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/structure_only/structure_only_solver.h"
#include "g2o/stuff/sampler.h"
#include "g2o/types/sba/types_six_dof_expmap.h"

using namespace Eigen;

#include <cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;

#include "Config.h"

typedef struct tsTrf
{
    Mat matT;
    double tx;
    double ty;
    double Rz;
} tsTrf;

typedef struct tsPolygon
{
    vector<Point2d> vPtsI;
    vector<Point2d> vPtsI_orig;
    vector<Point3d> vPtsW;
    tsTrf trf;
    double dt; // timestep
}tsPolygon;

typedef struct tsProblemConfig
{
    int iNumPolygons;
    int iNumPtsPerPolygon;
    double dHeightA;
    bool bUseAxToInit;
    bool bUseAyToInit;
    bool bUseAzToInit;
}tsProblemConfig;


class DataReader
{
    public:
        DataReader();
        ~DataReader();

        void checkNode(FileNode fn, string sNode);
        void setFileGeneratedPts2d3dInFlow(string sFile, vector<vector<tsPolygon>> &vvPolygonsInFlow, int iMaxNumPolygonsToConsider, bool bDebug = false);
        void getTruePoints(tsPolygon polygon, vector<Vector3d> &true_points, bool bDebug = false);
        void getObservations(vector<tsPolygon> vPolygonsInFlow, int iStartPolygonIdx, int iNumPolygonsToConsider, vector<Vector2d> &observations, bool bDebug = false);
        void getTrfs(vector<tsPolygon> vPolygonsInFlow, int iStartPolygonIdx, int iNumPolygonsToConsider, vector<Mat> &vMatTrfs, bool bDebug = false);
        void setFileExtrinsics(string sFile, Mat &matP, Mat &matHgToI, Mat &matHiToG, bool bDebug = false);
        void setFileIntrinsics(string sFile, Mat &matK, Mat &matD, bool bDebug = false);

        void setParameters(string sFile, tsProblemConfig &problemConfig, 
            Mat &matInitParams, 
            Mat &matOptParams, 
            int iInvocationIdx, 
            bool bDebug = false);
        

    private:
        FileStorage _fs;
};

#endif // DATAREADER_H_
