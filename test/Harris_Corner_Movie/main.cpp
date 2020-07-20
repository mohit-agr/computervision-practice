#include "CSVParser.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
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
    cv::Mat srcImgConcat = cv::Mat::zeros(IMG_HEIGHT, 2 * IMG_WIDTH, CV_8U);
    cv::Mat srcImgLeft = cv::Mat::zeros(IMG_HEIGHT, IMG_WIDTH, CV_8U);
    cv::Mat srcImgLeftRGB = cv::Mat::zeros(IMG_HEIGHT, IMG_WIDTH, CV_8UC3);
    cv::Mat srcImgRight = cv::Mat::zeros(IMG_HEIGHT, IMG_WIDTH, CV_8U);
    cv::Mat srcImgRightRGB = cv::Mat::zeros(IMG_HEIGHT, IMG_WIDTH, CV_8UC3);
    cv::Mat dstLeft = cv::Mat::zeros(srcImgLeft.size(), CV_8U);
    cv::Mat dstRight = cv::Mat::zeros(srcImgRight.size(), CV_8U);
    cv::Mat dstLeft_norm, dstRight_norm, dstLeft_norm_scaled, dstRight_norm_scaled;

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
        //if (counter < 1200)
        //    continue;
        //std::cout << "Time Stamp [ns] : " << row[0] << " ";
        if (counter % 200 == 0) {
            std::cout << "Recorded : " << (float)counter / 20.0 << " sec" << std::endl;
        }
        std::cout << "File Name : " << LEFT_IMG_PATH_PREFIX + row[1] << "\n";
        srcImgLeft = cv::imread(LEFT_IMG_PATH_PREFIX + row[1], cv::IMREAD_GRAYSCALE);
        if (!srcImgLeft.data)
            continue;
        
        srcImgRight = cv::imread(RIGHT_IMG_PATH_PREFIX + row[1], cv::IMREAD_GRAYSCALE);
        // convert left and right monochrome image to color image (for color overlay)
        cv::cvtColor(srcImgLeft, srcImgLeftRGB, cv::COLOR_GRAY2BGR);
        cv::cvtColor(srcImgRight, srcImgRightRGB, cv::COLOR_GRAY2BGR);
        // detect harris corner (on monochrome images only) and save it to dstLeft and dstRight  
        cv::cornerHarris(srcImgLeft, dstLeft, blockSize, apertureSize, k);
        cv::cornerHarris(srcImgRight, dstRight, blockSize, apertureSize, k);
        // Normalized and scale images with features
        cv::normalize(dstLeft, dstLeft_norm, 0, 255, cv::NORM_MINMAX, CV_8U, cv::Mat());
        cv::convertScaleAbs(dstLeft_norm, dstLeft_norm_scaled);
        cv::normalize(dstRight, dstRight_norm, 0, 255, cv::NORM_MINMAX, CV_8U, cv::Mat());
        cv::convertScaleAbs(dstRight_norm, dstRight_norm_scaled);

        // draw overlay on left image
        for (int i = 0; i < dstLeft_norm.rows; i++)
        {
            for (int j = 0; j < dstLeft_norm.cols; j++)
            {
                if ((int)dstLeft_norm.at<uint8_t>(i, j) > thresh)
                {
                    circle(srcImgLeftRGB, cv::Point(j, i), 5, cv::Scalar(0, 0, 255), 2, 8, 0);
                }
            }
        }
        // draw overlay on right image
        for (int i = 0; i < dstRight_norm.rows; i++)
        {
            for (int j = 0; j < dstRight_norm.cols; j++)
            {
                if ((int)dstRight_norm.at<uint8_t>(i, j) > thresh)
                {
                    circle(srcImgRightRGB, cv::Point(j, i), 5, cv::Scalar(0, 0, 255), 2, 8, 0);
                }
            }
        }
        // Horizontally concat left and right images


        cv::hconcat(srcImgLeftRGB, srcImgRightRGB, srcImgConcat);
        //cv::imshow("text", srcImgConcat);
        //cv::waitKey();
        // write images with features into video stream
        outputVideo.write(srcImgConcat);
    }

    std::cout << "Recording End." << std::endl;
    outputVideo.release();
}
