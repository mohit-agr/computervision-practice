#include "CSVParser.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/core/cvstd.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp> // for imread()
#include <iostream>

#define FILE_PATH "../../dataset/MH_01_easy/mav0/cam0/data.csv"
#define VIDEO_FILE_PATH "MH_01_easy_mav0_cam0.avi"
#define LEFT_IMG_PATH_PREFIX "../../dataset/MH_01_easy/mav0/cam0/data/"
#define RIGHT_IMG_PATH_PREFIX "../../dataset/MH_01_easy/mav0/cam1/data/"
#define IMG_WIDTH 752
#define IMG_HEIGHT 480

#define GENERATE_DISPARITY

#ifdef GENERATE_DISPARITY
// SGBM parameters
int windowSize = 9;
int minDisparity = 32;
int numDisparity = 112 - minDisparity;
int maxBlockSize = 11;
#endif 

int main()
{
    std::ifstream file(FILE_PATH);
    CSVParser row;

    cv::VideoWriter outputVideo;
    outputVideo.open(VIDEO_FILE_PATH, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 20, cv::Size(2 * IMG_WIDTH, IMG_HEIGHT), true);

    if (!outputVideo.isOpened()) {
        std::cout << "Could not open the output video for write: " << std::endl;
        return -1;
    }
    
    cv::Mat srcImgConcat;
    cv::Mat disparity_map, disparity_map_8U;
    cv::Mat srcImgLeft = cv::Mat::zeros(IMG_HEIGHT, IMG_WIDTH, CV_8U);
    cv::Mat srcImgRight = cv::Mat::zeros(IMG_HEIGHT, IMG_WIDTH, CV_8U);
    cv::Mat dstLeft = cv::Mat::zeros(srcImgLeft.size(), CV_8U);
    cv::Mat dstRight = cv::Mat::zeros(srcImgRight.size(), CV_8U);

    // ORB Feature Detector
    cv::Ptr<cv::ORB> detector = cv::ORB::create(400);
    std::vector<cv::KeyPoint> keypoints_left, keypoints_right;
    #ifndef USE_FLANN
    // Brute Force matcher
    cv::Ptr<cv::BFMatcher> bf = cv::BFMatcher::create(cv::NORM_HAMMING, true);
    #else
    cv::FlannBasedMatcher bf = cv::FlannBasedMatcher(cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2));

    #endif
    std::vector<cv::DMatch> matches;

    // Camera parameters
    // EuROC Camera Extrinsic parameters with respect to the base frame. Left is cam0. Right is cam1.
    cv::Mat Camera_Extrinsic_Left = (cv::Mat1d(4, 4) << 0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975, \
        0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768, \
        - 0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949, \
        0.0, 0.0, 0.0, 1.0);
    cv::Mat Camera_Extrinsic_Right = (cv::Mat1d(4, 4) << 0.0125552670891, -0.999755099723, 0.0182237714554, -0.0198435579556, \
        0.999598781151, 0.0130119051815, 0.0251588363115, 0.0453689425024, \
        - 0.0253898008918, 0.0179005838253, 0.999517347078, 0.00786212447038, \
        0.0, 0.0, 0.0, 1.0);
    cv::Mat Camera_Extrinsic = (Camera_Extrinsic_Left.inv() * Camera_Extrinsic_Right).inv();
    cv::Mat Camera_R = Camera_Extrinsic(cv::Rect(0, 0, 3, 3));
    cv::Mat Camera_T = Camera_Extrinsic(cv::Rect(3, 0, 1, 3));
    
    // Camera matrix (Intrinsic parameters)
    cv::Mat Camera_Matrix_Left = (cv::Mat1d(3, 3) << 458.654, 0, 367.215, 0, 457.296, 248.375, 0.0, 0.0, 1.0);
    cv::Mat Camera_Matrix_Right = (cv::Mat1d(3, 3) << 457.587, 0, 379.999, 0, 456.134, 255.238, 0.0, 0.0, 1.0);
    
    // distortion coefficient
    cv::Mat distortion_coeff_left = (cv::Mat1d(1, 5) << -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05, 0);
    cv::Mat distortion_coeff_right = (cv::Mat1d(1, 5) << -0.28368365, 0.07451284, -0.00010473, -3.55590700e-05, 0);

    // stereo rectify coefficients
    cv::Mat R1 = cv::Mat1d(3, 3);
    cv::Mat R2 = cv::Mat1d(3, 3);
    cv::Mat P1 = cv::Mat1d(3, 4);
    cv::Mat P2 = cv::Mat1d(3, 4);
    cv::Mat Q = cv::Mat1d(4, 4);
    cv::Rect validRoi[2];

    int rect_alpha = 0; // alpha=1 for the whole image, but if there is too much warp, make it less.
    cv::stereoRectify(Camera_Matrix_Left, distortion_coeff_left, Camera_Matrix_Right, \
        distortion_coeff_right, cv::Size(IMG_WIDTH, IMG_HEIGHT), Camera_R, Camera_T, R1, R2, P1, P2, \
        Q, 0, rect_alpha, cv::Size(IMG_WIDTH, IMG_HEIGHT), &validRoi[0]);

    // rectify images
    cv::Mat map11, map12, map21, map22;
    cv::initUndistortRectifyMap(Camera_Matrix_Left, distortion_coeff_left, R1, P1, cv::Size(IMG_WIDTH, IMG_HEIGHT), CV_16SC2, map11, map12);
    cv::initUndistortRectifyMap(Camera_Matrix_Right, distortion_coeff_right, R2, P2, cv::Size(IMG_WIDTH, IMG_HEIGHT), CV_16SC2, map21, map22);

    unsigned int counter = 0;

    std::cout << "Start Recording..." << std::endl;

    cv::Mat img_left_rect, img_right_rect;
    while (file >> row)
    {
        counter++;
        //std::cout << "Time Stamp [ns] : " << row[0] << " ";
        if (counter % 20 == 0) {
            std::cout << "Recorded : " << (float)counter / 20.0 << " sec" << std::endl;
        }
        std::cout << "File Name : " << LEFT_IMG_PATH_PREFIX + row[1] << "\n";

        srcImgLeft = cv::imread(LEFT_IMG_PATH_PREFIX + row[1], cv::IMREAD_GRAYSCALE);
        srcImgRight = cv::imread(RIGHT_IMG_PATH_PREFIX + row[1], cv::IMREAD_GRAYSCALE);
        if (!srcImgLeft.data || !srcImgRight.data)
            continue;

        cv::remap(srcImgLeft, img_left_rect, map11, map12, cv::INTER_LINEAR);
        cv::remap(srcImgRight, img_right_rect, map21, map22, cv::INTER_LINEAR);

        srcImgLeft = img_left_rect;
        srcImgRight = img_right_rect;

        // detect ORB features and save it
        detector->detectAndCompute(srcImgLeft, cv::Mat(), keypoints_left, dstLeft);
        detector->detectAndCompute(srcImgRight, cv::Mat(), keypoints_right, dstRight);
        
        // BF Match descriptors
        #ifndef USE_FLANN
        bf->match(dstLeft, dstRight, matches);
        #endif
        std::sort(matches.begin(), matches.end());
        matches.resize(100);
        
        cv::drawMatches(srcImgLeft, keypoints_left, srcImgRight, keypoints_right, matches, \
            srcImgConcat, cv::Scalar::all(-1), cv::Scalar::all(-1), cv::Mat(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
        
        // write images with features into video stream
        outputVideo.write(srcImgConcat);
        cv::imshow("ORB Feature Match", srcImgConcat);
        cv::waitKey(1);

        #ifdef GENERATE_DISPARITY
        // Generate disparity map
        cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(minDisparity, numDisparity, windowSize, \
            8 * windowSize * windowSize, 32 * windowSize * windowSize, \
            1, 0, 10, 150, 32, cv::StereoSGBM::MODE_HH);
        sgbm->compute(srcImgLeft, srcImgRight, disparity_map);
        // Normalize image 0 to 255.
        disparity_map.convertTo(disparity_map_8U, CV_8U, 255 / (16.0 * sgbm->getNumDisparities()), \
            - sgbm->getMinDisparity() / sgbm->getNumDisparities());
        cv::imshow("Disparity Map", disparity_map_8U);
        cv::waitKey(0);
        #endif
    }

    std::cout << "Recording End." << std::endl;
    outputVideo.release();
}
