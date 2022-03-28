#include "DataReader.h"



DataReader::DataReader(){}


DataReader::~DataReader(){}

void DataReader::checkNode(FileNode fn, string sNode)
{
    if (fn.empty())
        cerr << "Node " << sNode << " is absent." << endl;
}

void DataReader::setFileGeneratedPts2d3dInFlow(string sFile, vector<vector<tsPolygon>> &vvPolygonsInFlow, bool bDebug)
{
    vvPolygonsInFlow.clear();

    _fs.open(sFile, FileStorage::READ);
    if (!_fs.isOpened())
        cerr << "Unable to read: " << sFile << endl;

    string sNode = "frameNo";
    FileNode fn = _fs[sNode];
    checkNode(fn, sNode);
    int _iFrameNo = (int)fn;

    sNode = "iOffset_TL_X";
    fn = _fs[sNode];
    checkNode(fn, sNode);
    int iOffset_TL_X = (int)fn;

    sNode = "iOffset_TL_Y";
    fn = _fs[sNode];
    checkNode(fn, sNode);
    int iOffset_TL_Y = (int)fn;

    sNode = "iNumPts2d3dInFlow";
    fn = _fs[sNode];
    checkNode(fn, sNode);
    int iNumPts2d3dInFlow = (int)fn;

    sNode = "iNumPts";
    fn = _fs[sNode];
    checkNode(fn, sNode);
    int iNumPts = (int)fn;

    int _iNumFramesForLMOpt = 3;
    Mat matImg = Mat(1080, (iNumPts + _iNumFramesForLMOpt + 2) * 250, CV_8UC3);

    sNode = "iNumFlows";
    fn = _fs[sNode];
    checkNode(fn, sNode);
    int iNumFlows = (int)fn;

    vector<Scalar> vColors;
    vColors.push_back(COLOR_RED);
    vColors.push_back(COLOR_GREEN);
    vColors.push_back(COLOR_YELLOW);

    for (int iIdxPts2d3d = 0; iIdxPts2d3d < iNumPts2d3dInFlow; iIdxPts2d3d++)
    {
        vector<tsPolygon> vPolygonsInFlow;
        matImg.setTo(0);

        sNode = "pts2d3dInFlow" + to_string(iIdxPts2d3d);
        fn = _fs[sNode];
        checkNode(fn, sNode);

        sNode = "numPolygons";
        FileNode fn2 = fn[sNode];
        checkNode(fn2, sNode);
        int iNumPolygons = (int)fn2;

        int iMaxNumPolygonsToConsider = 100;
        for (int iPolygonIdx = 0; iPolygonIdx < iNumPolygons; iPolygonIdx++)
        {
            if (iPolygonIdx > iMaxNumPolygonsToConsider)
                break;

            string sNode = "polygon" + to_string(iPolygonIdx);
            fn2 = fn[sNode];
            checkNode(fn2, sNode);

            tsPolygon polygon;
            FileNode fn3;
            for (int iIdx = 0; iIdx < iNumPts; iIdx++)
            {
                Point2d ptI;
                Point3d ptW;
                sNode = "x" + to_string(iIdx);
                fn3 = fn2[sNode];
                checkNode(fn3, sNode);
                ptI.x = (double)fn3 + iOffset_TL_X;

                sNode = "y" + to_string(iIdx);
                fn3 = fn2[sNode];
                checkNode(fn3, sNode);
                ptI.y = (double)fn3 + iOffset_TL_Y;

                sNode = "X" + to_string(iIdx);
                fn3 = fn2[sNode];
                checkNode(fn3, sNode);
                ptW.x = (double)fn3;

                sNode = "Y" + to_string(iIdx);
                fn3 = fn2[sNode];
                checkNode(fn3, sNode);
                ptW.y = (double)fn3;

                sNode = "Z" + to_string(iIdx);
                fn3 = fn2[sNode];
                checkNode(fn3, sNode);
                ptW.z = (double)fn3;

                polygon.vPtsI.push_back(ptI);
                polygon.vPtsI_orig.push_back(ptI);
                polygon.vPtsW.push_back(ptW);
            }

            // Get Trf
            tsTrf trf;
            sNode = "matT";
            fn3 = fn2[sNode];
            checkNode(fn3, sNode);
            fn3 >> trf.matT;

            sNode = "tx";
            fn3 = fn2[sNode];
            checkNode(fn3, sNode);
            trf.tx = (double)fn3;

            sNode = "ty";
            fn3 = fn2[sNode];
            checkNode(fn3, sNode);
            trf.ty = (double)fn3;

            sNode = "RzRad";
            fn3 = fn2[sNode];
            checkNode(fn3, sNode);
            trf.Rz = (double)fn3;

            sNode = "dt";
            fn3 = fn2[sNode];
            checkNode(fn3, sNode);
            polygon.dt = (double)fn3;

            polygon.trf = trf;
            vPolygonsInFlow.push_back(polygon);

            // Draw the polygon
            for (int iPtIdx = 0;
                 iPtIdx < (int)polygon.vPtsI.size() - 1; iPtIdx++)
            {
                if (iPtIdx == 0)
                    line(matImg, polygon.vPtsI.front(), polygon.vPtsI.back(),
                         vColors[iPolygonIdx % 3], 1);
                line(matImg, polygon.vPtsI[iPtIdx], polygon.vPtsI[iPtIdx + 1],
                     vColors[iPolygonIdx % 3], 1);
            }

            putText(matImg, "frame no. " + to_string(_iFrameNo), Point(10, 100),
                    FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0, 0, 255), 2);
            putText(matImg, to_string(iNumPolygons) + " polygons",
                    Point(10, 150), FONT_HERSHEY_COMPLEX_SMALL, 1,
                    Scalar(0, 0, 255), 2);
        }
        vvPolygonsInFlow.push_back(vPolygonsInFlow);

        bool bDrawArrowedLine = true;
        if (bDrawArrowedLine)
        {
            for (int iPolygonIdx = 0; iPolygonIdx < iNumPolygons - 1; iPolygonIdx++)
                arrowedLine(matImg, vPolygonsInFlow[iPolygonIdx].vPtsI[0],
                            vPolygonsInFlow[iPolygonIdx + 1].vPtsI[0],
                            COLOR_RED, 1);
        }
        namedWindow("GeneratedPolygons", 0);
        imshow("GeneratedPolygons", matImg);
        waitKey(0);
    }

    if (_fs.isOpened())
        _fs.release();

    if (matImg.data)
        matImg.release();    
}


void DataReader::setFileExtrinsics(string sFile, Mat &matP, Mat &matHgToI, Mat &matHiToG, bool bDebug)
{
    _fs.open(sFile, FileStorage::READ);
    if (!_fs.isOpened())
    {
        cerr << "Unable to read: " << sFile << endl;
        return;
    }

    string sNode = "matP";
    FileNode fn = _fs[sNode];
    checkNode(fn, sNode);
    fn >> matP;

    sNode = "matHgToI";
    fn = _fs[sNode];
    checkNode(fn, sNode);
    fn >> matHgToI;

    sNode = "matHiToG";
    fn = _fs[sNode];
    checkNode(fn, sNode);
    fn >> matHiToG;

    if(_fs.isOpened()) _fs.release();
}

void DataReader::setFileIntrinsics(string sFile, Mat &matK, Mat &matD, bool bDebug)
{
    _fs.open(sFile, FileStorage::READ);
    if (!_fs.isOpened())
    {
        cerr << "Unable to read: " << sFile << endl;
        return;
    }

    string sNode = "camera_matrix";
    FileNode fn = _fs[sNode];
    checkNode(fn, sNode);
    fn >> matK;

    sNode = "distortion_coefficients";
    fn = _fs[sNode];
    checkNode(fn, sNode);
    fn >> matD;

    if(_fs.isOpened()) _fs.release();
}

// Given polygons in flow, get the 3D world points
void DataReader::getTruePoints(vector<tsPolygon> vPolygonsInFlow, int iNumPolygonsToConsider, vector<Vector3d> &true_points, bool bDebug)
{
    true_points.clear();
    int iNumPolygons = (int)vPolygonsInFlow.size();
    for(int iPolygonIdx = 0; iPolygonIdx < iNumPolygons; iPolygonIdx++)
    {
        if(iPolygonIdx >= iNumPolygonsToConsider) break;
        tsPolygon polygon = vPolygonsInFlow[iPolygonIdx];
        int iNumPts = (int)polygon.vPtsW.size();
        for(int iPtIdx = 0; iPtIdx < iNumPts; iPtIdx++)
        {
            Point3d pt = polygon.vPtsW[iPtIdx];
            true_points.push_back(Vector3d(pt.x, pt.y, pt.z));
            if(bDebug) cout << "iPolygonIdx: " << iPolygonIdx << " iPtIdx: " << iPtIdx << "\t" << pt.x << ", " << pt.y << ", " << pt.z << endl;
        }
    }
}

void DataReader::getObservations(vector<tsPolygon> vPolygonsInFlow, int iNumPolygonsToConsider, vector<Vector2d> &observations, bool bDebug)
{
    observations.clear();
    int iNumPolygons = (int)vPolygonsInFlow.size();
    for(int iPolygonIdx = 0; iPolygonIdx < iNumPolygons; iPolygonIdx++)
    {
        if(iPolygonIdx >= iNumPolygonsToConsider) break;
        tsPolygon polygon = vPolygonsInFlow[iPolygonIdx];
        int iNumPts = (int)polygon.vPtsW.size();
        for(int iPtIdx = 0; iPtIdx < iNumPts; iPtIdx++)
        {
            Point2d pt = polygon.vPtsI[iPtIdx];
            observations.push_back(Vector2d(pt.x, pt.y));
            if(bDebug) cout << "iPolygonIdx: " << iPolygonIdx << " iPtIdx: " << iPtIdx << "\t" << pt.x << ", " << pt.y << endl;
        }
    }
}


// Given polygons in flow, get the 3D world points
void DataReader::getTrfs(vector<tsPolygon> vPolygonsInFlow, int iNumPolygonsToConsider, vector<Mat> &vMatTrfs, bool bDebug)
{
    vMatTrfs.clear();
    int iNumPolygons = (int)vPolygonsInFlow.size();
    for(int iPolygonIdx = 0; iPolygonIdx < iNumPolygons; iPolygonIdx++)
    {
        if(iPolygonIdx >= iNumPolygonsToConsider) break;
        tsPolygon polygon = vPolygonsInFlow[iPolygonIdx];
        Mat matT = polygon.trf.matT;
        vMatTrfs.push_back(matT); // NOTE: First matrix is zero matrix, not to be used.
    }
}

