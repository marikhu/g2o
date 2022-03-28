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

#define COLOR_RED Scalar(0,0,255)
#define COLOR_GREEN Scalar(0,255,0)
#define COLOR_BLUE Scalar(255,0,0)
#define COLOR_YELLOW Scalar(0,255,255)

#define YML_GENERATED_PTS2D3D_FLOW "input/GeneratedDtInMotion/Thu_1_Apr_2021_04-45-56/ptsForDtInFlow.yml"
#define YML_EXTRNSICS "input/GeneratedDtInMotion/Thu_1_Apr_2021_04-45-56/userdefextrinsics.yml" 
#define YML_INTRINSICS "input/GeneratedDtInMotion/Thu_1_Apr_2021_04-45-56/userdefintrinsics.yml" 

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


class DataReader
{
    public:
        DataReader();
        ~DataReader();

        void checkNode(FileNode fn, string sNode);
        void setFileGeneratedPts2d3dInFlow(string sFile, vector<vector<tsPolygon>> &vvPolygonsInFlow, bool bDebug = false);
        void getTruePoints(vector<tsPolygon> vPolygonsInFlow, int iNumPolygonsToConsider, vector<Vector3d> &true_points, bool bDebug = false);
        void getObservations(vector<tsPolygon> vPolygonsInFlow, int iNumPolygonsToConsider, vector<Vector2d> &observations, bool bDebug = false);
        void getTrfs(vector<tsPolygon> vPolygonsInFlow, int iNumPolygonsToConsider, vector<Mat> &vMatTrfs, bool bDebug = false);
        void setFileExtrinsics(string sFile, Mat &matP, Mat &matHgToI, Mat &matHiToG, bool bDebug = false);
        void setFileIntrinsics(string sFile, Mat &matK, Mat &matD, bool bDebug = false);
        

    private:
        FileStorage _fs;
};

#endif // DATAREADER_H_