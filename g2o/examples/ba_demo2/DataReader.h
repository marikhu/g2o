#ifndef DATAREADER_H_
#define DATAREADER_H_

#include <vector>
#include <iostream>
#include <string>
using namespace std;

#include <cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;

#define YML_GENERATED_PTS2D3D_FLOW "input/GeneratedDtInMotion/Thu_1_Apr_2021_04-45-56/ptsForDtInFlow.yml"
#define COLOR_RED Scalar(0,0,255)
#define COLOR_GREEN Scalar(0,255,0)
#define COLOR_YELLOW Scalar(0,255,255)


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

    private:
        FileStorage _fs;
};

#endif // DATAREADER_H_