#include "opencv2/core.hpp"
#include <opencv2/core/utility.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include <cctype>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <iostream>

#include "CSVParser.h"

using namespace std;

#define FILE_PATH "../../dataset/cam_checkerboard/mav0/cam0/data_reduced.csv"
#define LEFT_IMG_PATH_PREFIX "../../dataset/cam_checkerboard/mav0/cam0/data/"

enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };
enum Pattern { CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };

static bool runAndSave(const string& outputFilename, \
    const vector<vector<cv::Point2f> >& imagePoints, \
    cv::Size imageSize, cv::Size boardSize, Pattern patternType, float squareSize, \
    float grid_width, bool release_object, \
    float aspectRatio, int flags, cv::Mat& cameraMatrix, \
    cv::Mat& distCoeffs, bool writeExtrinsics, bool writePoints, bool writeGrid);

static void saveCameraParams(const string& filename, \
    cv::Size imageSize, cv::Size boardSize, \
    float squareSize, float aspectRatio, int flags, \
    const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, \
    const vector<cv::Mat>& rvecs, const vector<cv::Mat>& tvecs, \
    const vector<float>& reprojErrs, \
    const vector<vector<cv::Point2f> >& imagePoints, \
    const vector<cv::Point3f>& newObjPoints, \
    double totalAvgErr);

static bool runCalibration(vector<vector<cv::Point2f> > imagePoints, \
    cv::Size imageSize, cv::Size boardSize, Pattern patternType, \
    float squareSize, float aspectRatio, \
    float grid_width, bool release_object, \
    int flags, cv::Mat& cameraMatrix, cv::Mat& distCoeffs, \
    vector<cv::Mat>& rvecs, vector<cv::Mat>& tvecs, \
    vector<float>& reprojErrs, \
    vector<cv::Point3f>& newObjPoints, \
    double& totalAvgErr);

static double computeReprojectionErrors( \
    const vector<vector<cv::Point3f> >& objectPoints, \
    const vector<vector<cv::Point2f> >& imagePoints, \
    const vector<cv::Mat>& rvecs, const vector<cv::Mat>& tvecs, \
    const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, \
    vector<float>& perViewErrors);

static void calcChessboardCorners(cv::Size boardSize, float squareSize, vector<cv::Point3f>& corners, Pattern patternType);

int main(void) {

	std::ifstream file(FILE_PATH);
	CSVParser row;

	cv::Size boardSize, imageSize;
	float squareSize = 0.06; // meters
	float	aspectRatio = 1;
	cv::Mat cameraMatrix, distCoeffs;
	std::string outputFilename;
    float winSize = 11;
    vector<vector<cv::Point2f> > imagePoints;

    int mode = CAPTURING;
	
	Pattern pattern = CHESSBOARD;

	// TODO : get correct input
	boardSize.width = 6;
	boardSize.height = 7;
	
	while (file >> row)
	{
        cv::Mat view = cv::imread(LEFT_IMG_PATH_PREFIX + row[1], cv::IMREAD_GRAYSCALE);
        if (!view.data)
            continue;
        imageSize = view.size();
        vector<cv::Point2f> pointbuf;
        // find corners on the chess board
        bool found = cv::findChessboardCorners(view, boardSize, pointbuf, \
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
        // improve the corner's accuracy
        if (found) {
            cv::cornerSubPix(view, pointbuf, cv::Size(winSize, winSize), \
                cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.0001));

        }

        if (mode == CAPTURING && found) {
            imagePoints.push_back(pointbuf);
        }

        if (found) {
            drawChessboardCorners(view, boardSize, cv::Mat(pointbuf), found);
        }

        cv::imshow("Image View", view);
        cv::waitKey(1);

	}

	return 0;
}

static bool runAndSave(const string& outputFilename,
    const vector<vector<cv::Point2f> >& imagePoints,
    cv::Size imageSize, cv::Size boardSize, Pattern patternType, float squareSize,
    float grid_width, bool release_object,
    float aspectRatio, int flags, cv::Mat& cameraMatrix,
    cv::Mat& distCoeffs, bool writeExtrinsics, bool writePoints, bool writeGrid)
{
    vector<cv::Mat> rvecs, tvecs;
    vector<float> reprojErrs;
    double totalAvgErr = 0;
    vector<cv::Point3f> newObjPoints;

    bool ok = runCalibration(imagePoints, imageSize, boardSize, patternType, squareSize,
        aspectRatio, grid_width, release_object, flags, cameraMatrix, distCoeffs,
        rvecs, tvecs, reprojErrs, newObjPoints, totalAvgErr);
    printf("%s. avg reprojection error = %.7f\n",
        ok ? "Calibration succeeded" : "Calibration failed",
        totalAvgErr);

    if (ok)
        saveCameraParams(outputFilename, imageSize,
            boardSize, squareSize, aspectRatio,
            flags, cameraMatrix, distCoeffs,
            writeExtrinsics ? rvecs : vector<cv::Mat>(),
            writeExtrinsics ? tvecs : vector<cv::Mat>(),
            writeExtrinsics ? reprojErrs : vector<float>(),
            writePoints ? imagePoints : vector<vector<cv::Point2f> >(),
            writeGrid ? newObjPoints : vector<cv::Point3f>(),
            totalAvgErr);
    return ok;
}

static bool runCalibration(vector<vector<cv::Point2f> > imagePoints,
    cv::Size imageSize, cv::Size boardSize, Pattern patternType,
    float squareSize, float aspectRatio,
    float grid_width, bool release_object,
    int flags, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
    vector<cv::Mat>& rvecs, vector<cv::Mat>& tvecs,
    vector<float>& reprojErrs,
    vector<cv::Point3f>& newObjPoints,
    double& totalAvgErr)
{
    cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    if (flags & cv::CALIB_FIX_ASPECT_RATIO)
        cameraMatrix.at<double>(0, 0) = aspectRatio;

    distCoeffs = cv::Mat::zeros(8, 1, CV_64F);

    vector<vector<cv::Point3f> > objectPoints(1);
    calcChessboardCorners(boardSize, squareSize, objectPoints[0], patternType);
    objectPoints[0][boardSize.width - 1].x = objectPoints[0][0].x + grid_width;
    newObjPoints = objectPoints[0];

    objectPoints.resize(imagePoints.size(), objectPoints[0]);

    double rms;
    int iFixedPoint = -1;
    if (release_object)
        iFixedPoint = boardSize.width - 1;
    rms = calibrateCameraRO(objectPoints, imagePoints, imageSize, iFixedPoint,
        cameraMatrix, distCoeffs, rvecs, tvecs, newObjPoints,
        flags | cv::CALIB_FIX_K3 | cv::CALIB_USE_LU);
    printf("RMS error reported by calibrateCamera: %g\n", rms);

    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

    if (release_object) {
        cout << "New board corners: " << endl;
        cout << newObjPoints[0] << endl;
        cout << newObjPoints[boardSize.width - 1] << endl;
        cout << newObjPoints[boardSize.width * (boardSize.height - 1)] << endl;
        cout << newObjPoints.back() << endl;
    }

    objectPoints.clear();
    objectPoints.resize(imagePoints.size(), newObjPoints);
    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
        rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

    return ok;
}

static void saveCameraParams(const string& filename,
    cv::Size imageSize, cv::Size boardSize,
    float squareSize, float aspectRatio, int flags,
    const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
    const vector<cv::Mat>& rvecs, const vector<cv::Mat>& tvecs,
    const vector<float>& reprojErrs,
    const vector<vector<cv::Point2f> >& imagePoints,
    const vector<cv::Point3f>& newObjPoints,
    double totalAvgErr)
{
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);

    time_t tt;
    time(&tt);
    struct tm* t2 = localtime(&tt);
    char buf[1024];
    strftime(buf, sizeof(buf) - 1, "%c", t2);

    fs << "calibration_time" << buf;

    if (!rvecs.empty() || !reprojErrs.empty())
        fs << "nframes" << (int)std::max(rvecs.size(), reprojErrs.size());
    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;
    fs << "board_width" << boardSize.width;
    fs << "board_height" << boardSize.height;
    fs << "square_size" << squareSize;

    if (flags & cv::CALIB_FIX_ASPECT_RATIO)
        fs << "aspectRatio" << aspectRatio;

    if (flags != 0)
    {
        sprintf(buf, "flags: %s%s%s%s",
            flags & cv::CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
            flags & cv::CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
            flags & cv::CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
            flags & cv::CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "");
        //cvWriteComment( *fs, buf, 0 );
    }

    fs << "flags" << flags;

    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;

    fs << "avg_reprojection_error" << totalAvgErr;
    if (!reprojErrs.empty())
        fs << "per_view_reprojection_errors" << cv::Mat(reprojErrs);

    if (!rvecs.empty() && !tvecs.empty())
    {
        CV_Assert(rvecs[0].type() == tvecs[0].type());
        cv::Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
        for (int i = 0; i < (int)rvecs.size(); i++)
        {
            cv::Mat r = bigmat(cv::Range(i, i + 1), cv::Range(0, 3));
            cv::Mat t = bigmat(cv::Range(i, i + 1), cv::Range(3, 6));

            CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
            CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
            //*.t() is MatExpr (not Mat) so we can use assignment operator
            r = rvecs[i].t();
            t = tvecs[i].t();
        }
        //cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
        fs << "extrinsic_parameters" << bigmat;
    }

    if (!imagePoints.empty())
    {
        cv::Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
        for (int i = 0; i < (int)imagePoints.size(); i++)
        {
            cv::Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
            cv::Mat imgpti(imagePoints[i]);
            imgpti.copyTo(r);
        }
        fs << "image_points" << imagePtMat;
    }

    if (!newObjPoints.empty())
    {
        fs << "grid_points" << newObjPoints;
    }
}

static double computeReprojectionErrors(
    const vector<vector<cv::Point3f> >& objectPoints,
    const vector<vector<cv::Point2f> >& imagePoints,
    const vector<cv::Mat>& rvecs, const vector<cv::Mat>& tvecs,
    const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
    vector<float>& perViewErrors)
{
    vector<cv::Point2f> imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for (i = 0; i < (int)objectPoints.size(); i++)
    {
        projectPoints(cv::Mat(objectPoints[i]), rvecs[i], tvecs[i],
            cameraMatrix, distCoeffs, imagePoints2);
        err = cv::norm(cv::Mat(imagePoints[i]), cv::Mat(imagePoints2), cv::NORM_L2);
        int n = (int)objectPoints[i].size();
        perViewErrors[i] = (float)std::sqrt(err * err / n);
        totalErr += err * err;
        totalPoints += n;
    }

    return std::sqrt(totalErr / totalPoints);
}

static void calcChessboardCorners(cv::Size boardSize, float squareSize, vector<cv::Point3f>& corners, Pattern patternType = CHESSBOARD)
{
    corners.resize(0);

    switch (patternType)
    {
    case CHESSBOARD:
    case CIRCLES_GRID:
        for (int i = 0; i < boardSize.height; i++)
            for (int j = 0; j < boardSize.width; j++)
                corners.push_back(cv::Point3f(float(j * squareSize),
                    float(i * squareSize), 0));
        break;

    case ASYMMETRIC_CIRCLES_GRID:
        for (int i = 0; i < boardSize.height; i++)
            for (int j = 0; j < boardSize.width; j++)
                corners.push_back(cv::Point3f(float((2 * j + i % 2) * squareSize),
                    float(i * squareSize), 0));
        break;

    default:
        CV_Error(cv::Error::StsBadArg, "Unknown pattern type\n");
    }
}

