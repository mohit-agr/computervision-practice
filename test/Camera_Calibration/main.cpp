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

#define MAKE_VIDEO
#define SHOW_IMG
#define IMG_WIDTH 752
#define IMG_HEIGHT 480

#define FILE_PATH "../../dataset/cam_checkerboard/mav0/cam0/data_reduced.csv"
#define LEFT_IMG_PATH_PREFIX "../../dataset/cam_checkerboard/mav0/cam0/data/"
#define RIGHT_IMG_PATH_PREFIX "../../dataset/cam_checkerboard/mav0/cam1/data/"

static void StereoCalib(cv::Size boardSize, float squareSize, \
	bool displaycorners_left, bool useCalibrated, bool showRectified);


int main(void) {

	cv::Size boardSize, imageSize;
	float squareSize = 0.06; // meters
	float	aspectRatio = 1;
	cv::Mat cameraMatrix, distCoeffs;
	std::string outputFilename;
    float winSize = 11;
    vector<vector<cv::Point2f> > imagePoints;

	boardSize.width = 7;
	boardSize.height = 6;
	
	StereoCalib(boardSize, squareSize, true, true, true);

	return 0;
}

static void StereoCalib(cv::Size boardSize, float squareSize, \
	bool displaycorners_left = false, bool useCalibrated = true, bool showRectified = true)
{
	std::ifstream file(FILE_PATH);
	CSVParser row;
	
	#ifdef MAKE_VIDEO
	cv::VideoWriter outputVideo;
	outputVideo.open("stereo_calibration.mp4", cv::VideoWriter::fourcc('M', 'P', '4', 'V'), 20, cv::Size(2 * IMG_WIDTH, IMG_HEIGHT), true);
	if (!outputVideo.isOpened()) {
		std::cout << "Could not open the output video for write: " << std::endl;
		return;
	}
	#endif

	bool first = true;
	int nimages = 0; // store no of good matches in stereo images. TODO : fix value now. Change it to get accurate value.
	std::vector<string> goodImageList;
	vector<vector<cv::Point2f> > imagePoints[2];
	vector<vector<cv::Point3f> > objectPoints;
	cv::Size imageSize;

	imagePoints[0].resize(100);
	imagePoints[1].resize(100);

	while (file >> row)
	{
		std::cout << "Reading file : " << row[1] << std::endl;
		cv::Mat img_left = cv::imread(LEFT_IMG_PATH_PREFIX + row[1], cv::IMREAD_GRAYSCALE);
		cv::Mat img_right = cv::imread(RIGHT_IMG_PATH_PREFIX + row[1], cv::IMREAD_GRAYSCALE);

		if (!img_left.data || !img_right.data)
			continue;

		imageSize = img_left.size();
		vector<cv::Point2f> &corners_left = imagePoints[0][nimages];
		vector<cv::Point2f> &corners_right = imagePoints[1][nimages];
		bool found_left = false;
		bool found_right = false;

		found_left = findChessboardCorners(img_left, boardSize, corners_left,
			cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);
		found_right = findChessboardCorners(img_right, boardSize, corners_right,
			cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);

		if (displaycorners_left) {
			cv::Mat cimg_left, cimg_right;
			cv::Mat cimg_concat = cv::Mat::zeros(img_left.rows, img_left.cols *2, CV_8U);
			cv::cvtColor(img_left, cimg_left, cv::COLOR_GRAY2BGR);
			cv::cvtColor(img_right, cimg_right, cv::COLOR_GRAY2BGR);

			if (found_left) {
				cv::cornerSubPix(img_left, corners_left, cv::Size(11, 11), cv::Size(-1, -1),
					cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
						30, 0.01));
				cv::drawChessboardCorners(cimg_left, boardSize, corners_left, found_left);
				
			}

			if (found_right) {
				cv::cornerSubPix(img_right, corners_right, cv::Size(11, 11), cv::Size(-1, -1),
					cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
						30, 0.01));
				cv::drawChessboardCorners(cimg_right, boardSize, corners_right, found_right);
				
			}

			if (found_left && found_right) {
				nimages++;
				goodImageList.push_back(row[1]);
			}

			cv::hconcat(cimg_left, cimg_right, cimg_concat);

			#ifdef SHOW_IMG
				imshow("Corners_Stereo", cimg_concat);

				if (first) {
					cv::waitKey(0);
					first = false;
				}
				else {
					cv::waitKey(1);
				}
			#endif

			#ifdef MAKE_VIDEO
				outputVideo.write(cimg_concat);
			#endif
		}	
	}

	if (nimages < 2) {
		std::cout << "Not enought matched images. Try with different data." << std::endl;
		return;
	}
	
	objectPoints.resize(nimages);
	imagePoints[0].resize(nimages);
	imagePoints[1].resize(nimages);

	for (int i = 0; i < nimages; i++)
	{
		for (int j = 0; j < boardSize.height; j++)
			for (int k = 0; k < boardSize.width; k++)
				objectPoints[i].push_back(cv::Point3f(k * squareSize, j * squareSize, 0));

	}

	cv::Mat cameraMatrix[2], distCoeffs[2];
	cameraMatrix[0] = initCameraMatrix2D(objectPoints, imagePoints[0], imageSize, 0);
	cameraMatrix[1] = initCameraMatrix2D(objectPoints, imagePoints[1], imageSize, 0);
	cv::Mat R, T, E, F;

	double rms = cv::stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
		cameraMatrix[0], distCoeffs[0],
		cameraMatrix[1], distCoeffs[1],
		imageSize, R, T, E, F,
		cv::CALIB_FIX_ASPECT_RATIO + cv::CALIB_ZERO_TANGENT_DIST + cv::CALIB_USE_INTRINSIC_GUESS +
		cv::CALIB_SAME_FOCAL_LENGTH + cv::CALIB_RATIONAL_MODEL + cv::CALIB_FIX_K3 + cv::CALIB_FIX_K4 + cv::CALIB_FIX_K5,
		cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-5));
	std::cout << "done with RMS error=" << rms << endl;

	// Calibration quality check
	double err = 0;
	int npoints = 0;
	vector<cv::Vec3f> lines[2];
	for (int i = 0; i < nimages; i++)
	{
		int npt = (int)imagePoints[0][i].size();
		cv::Mat imgpt[2];
		for (int k = 0; k < 2; k++)
		{
			imgpt[k] = cv::Mat(imagePoints[k][i]);
			cv::undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], cv::Mat(), cameraMatrix[k]);
			cv::computeCorrespondEpilines(imgpt[k], k + 1, F, lines[k]);
		}
		for (int j = 0; j < npt; j++)
		{
			double errij = fabs(imagePoints[0][i][j].x * lines[1][j][0] +
				imagePoints[0][i][j].y * lines[1][j][1] + lines[1][j][2]) +
				fabs(imagePoints[1][i][j].x * lines[0][j][0] +
					imagePoints[1][i][j].y * lines[0][j][1] + lines[0][j][2]);
			err += errij;
		}
		npoints += npt;
	}
	std::cout << "average epipolar err = " << err / npoints << endl;


	cv::Mat R1, R2, P1, P2, Q;
	cv::Rect validRoi[2];

	stereoRectify(cameraMatrix[0], distCoeffs[0],
		cameraMatrix[1], distCoeffs[1],
		imageSize, R, T, R1, R2, P1, P2, Q,
		cv::CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);
	std::cout << "R = " << R << std::endl;
	std::cout << "T = " << T << std::endl;
	std::cout << "R1 = " << R1 << std::endl;
	std::cout << "R2 = " << R2 << std::endl;
	std::cout << "P1 = " << P1 << std::endl;
	std::cout << "P2 = " << P2 << std::endl;
	std::cout << "Q = " << Q << std::endl;

	cv::Mat rmap[2][2];

	cv::initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
	cv::initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

	cv::Mat canvas;
	double sf;
	int w, h;

	sf = 600. / MAX(imageSize.width, imageSize.height);
	w = cvRound(imageSize.width * sf);
	h = cvRound(imageSize.height * sf);
	canvas.create(h, w * 2, CV_8UC3);


	for (int i = 0; i < nimages; i++)
	{
		cv::Mat img = cv::imread(LEFT_IMG_PATH_PREFIX + goodImageList[i], cv::IMREAD_GRAYSCALE), rimg, cimg;
		cv::remap(img, rimg, rmap[0][0], rmap[0][1], cv::INTER_LINEAR);
		cvtColor(rimg, cimg, cv::COLOR_GRAY2BGR);
		//cv::imshow("rectified left", rimg);
		cv::Mat canvasPart = canvas(cv::Rect(0, 0, w, h));
		cv::resize(cimg, canvasPart, canvasPart.size(), 0, 0, cv::INTER_AREA);
		cv::Rect vroi(cvRound(validRoi[0].x* sf), cvRound(validRoi[0].y* sf),
			cvRound(validRoi[0].width* sf), cvRound(validRoi[0].height* sf));
		cv::rectangle(canvasPart, vroi, cv::Scalar(0, 0, 255), 3, 8);
		for (int j = 0; j < canvas.rows; j += 16)
			cv::line(canvas, cv::Point(0, j), cv::Point(canvas.cols, j), cv::Scalar(0, 255, 0), 1, 8);
		imshow("rectified", canvas);

		// Right Image
		img = cv::imread(RIGHT_IMG_PATH_PREFIX + goodImageList[i], cv::IMREAD_GRAYSCALE);
		cv::remap(img, rimg, rmap[1][0], rmap[1][1], cv::INTER_LINEAR);
		//cvtColor(rimg, cimg, cv::COLOR_GRAY2BGR);
		cv::imshow("rectified Right", rimg);

		//cv::imshow("rectified", canvas);
		char c = (char)cv::waitKey(0);
		if (c == 27 || c == 'q' || c == 'Q')
			break;
	}

	

	std::cout << "All Points have been detected..." << std::endl;
}