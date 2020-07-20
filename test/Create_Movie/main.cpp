#include "CSVParser.h"
#include <opencv2/core/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui/highgui.hpp> // for imread()
#include <iostream>

#define FILE_PATH "../../dataset/MH_01_easy/mav0/cam0/data.csv"
#define VIDEO_FILE_PATH "MH_01_easy_mav0_cam0.avi"
#define LEFT_IMG_PATH_PREFIX "../../dataset/MH_01_easy/mav0/cam0/data/"
#define RIGHT_IMG_PATH_PREFIX "../../dataset/MH_01_easy/mav0/cam1/data/"
#define IMG_WIDTH 752
#define IMG_HEIGHT 480

int main()
{
    std::ifstream file(FILE_PATH);
    CSVParser row;

    std::vector<cv::Mat> images;
    cv::VideoWriter outputVideo;
    cv::Mat srcImgConcat = cv::Mat::zeros(IMG_HEIGHT, 2*IMG_WIDTH, CV_8U);
    cv::Mat srcImgLeft = cv::Mat::zeros(IMG_HEIGHT, IMG_WIDTH, CV_8U);
    cv::Mat srcImgRight = cv::Mat::zeros(IMG_HEIGHT, IMG_WIDTH, CV_8U);
    outputVideo.open(VIDEO_FILE_PATH, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 20, cv::Size(2*IMG_WIDTH, IMG_HEIGHT), false);

    
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
        //std::cout << "File Name : " << LEFT_IMG_PATH_PREFIX + row[1] << "\n";
        srcImgLeft = cv::imread(LEFT_IMG_PATH_PREFIX + row[1], cv::IMREAD_GRAYSCALE);
        srcImgRight = cv::imread(RIGHT_IMG_PATH_PREFIX + row[1], cv::IMREAD_GRAYSCALE);
        cv::hconcat(srcImgLeft, srcImgRight, srcImgConcat);
        outputVideo.write(srcImgConcat);
    }
    
    std::cout << "Recording End." << std::endl;
    outputVideo.release();
}
