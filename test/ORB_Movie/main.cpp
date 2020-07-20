#include "CSVParser.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/core/cvstd.hpp>
#include <opencv2/highgui/highgui.hpp> // for imread()
#include <iostream>

#define FILE_PATH "../../dataset/MH_01_easy/mav0/cam0/data.csv"
#define VIDEO_FILE_PATH "MH_01_easy_mav0_cam0.avi"
#define LEFT_IMG_PATH_PREFIX "../../dataset/MH_01_easy/mav0/cam0/data/"
#define RIGHT_IMG_PATH_PREFIX "../../dataset/MH_01_easy/mav0/cam1/data/"
#define IMG_WIDTH 752
#define IMG_HEIGHT 480

int blockSize = 2;
int apertureSize = 3;
double k = 0.04;
int thresh = 220;
int max_thresh = 255;

int main()
{
    std::ifstream file(FILE_PATH);
    CSVParser row;

    std::vector<cv::Mat> images;
    cv::VideoWriter outputVideo;
    cv::Mat srcImgConcat = cv::Mat::zeros(IMG_HEIGHT, IMG_WIDTH, CV_8U);
    cv::Mat srcImgLeft = cv::Mat::zeros(IMG_HEIGHT, IMG_WIDTH, CV_8U);
    cv::Mat srcImgRight = cv::Mat::zeros(IMG_HEIGHT, IMG_WIDTH, CV_8U);
    cv::Mat dstLeft = cv::Mat::zeros(srcImgLeft.size(), CV_8U);
    cv::Mat dstRight = cv::Mat::zeros(srcImgRight.size(), CV_8U);

    cv::Ptr<cv::ORB> detector = cv::ORB::create(400);
    std::vector<cv::KeyPoint> keypoints_left, keypoints_right;
    cv::Ptr<cv::BFMatcher> bf = cv::BFMatcher::create(cv::NORM_HAMMING, true);
    std::vector<cv::DMatch> matches;

    outputVideo.open(VIDEO_FILE_PATH, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 20, cv::Size(2 * IMG_WIDTH, IMG_HEIGHT), true);


    if (!outputVideo.isOpened()) {
        std::cout << "Could not open the output video for write: " << std::endl;
        return -1;
    }

    unsigned int counter = 0;

    std::cout << "Start Recording..." << std::endl;

    while (file >> row)
    {
        counter++;
        //std::cout << "Time Stamp [ns] : " << row[0] << " ";
        if (counter % 20 == 0) {
            std::cout << "Recorded : " << (float)counter / 20.0 << " sec" << std::endl;
        }
        std::cout << "File Name : " << LEFT_IMG_PATH_PREFIX + row[1] << "\n";
        srcImgLeft = cv::imread(LEFT_IMG_PATH_PREFIX + row[1], cv::IMREAD_GRAYSCALE);
        if (!srcImgLeft.data)
            continue;

        srcImgRight = cv::imread(RIGHT_IMG_PATH_PREFIX + row[1], cv::IMREAD_GRAYSCALE);
        // detect ORB features and save it
        detector->detectAndCompute(srcImgLeft, cv::Mat(), keypoints_left, dstLeft);
        detector->detectAndCompute(srcImgRight, cv::Mat(), keypoints_right, dstRight);
        
        // BF Match descriptors
        bf->match(dstLeft, dstRight, matches);

        std::sort(matches.begin(), matches.end());
        matches.resize(10);

        cv::drawMatches(srcImgLeft, keypoints_left, srcImgRight, keypoints_right, matches, \
            srcImgConcat, cv::Scalar::all(-1), cv::Scalar::all(-1), cv::Mat(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

        //cv::imshow("text", srcImgConcat);
        //cv::waitKey();
        // write images with features into video stream
        outputVideo.write(srcImgConcat);
    }

    std::cout << "Recording End." << std::endl;
    outputVideo.release();
}
