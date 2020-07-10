#include "StreamReader.hpp"

#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

void readImageFromFolder(const std::string &folder, std::vector<cv::Mat> *imageStream)
{
    std::fstream csv;
    csv.open(folder + "data.csv", std::ios::in);
    std::string ts, imgFile;
    std::getline(csv, ts);
    while (std::getline(csv, ts, ',')) {
        std::getline(csv, imgFile);
        const std::string file = folder + "data/" + ts + ".png";
        cv::Mat img = cv::imread(file);
        imageStream->push_back(img);
    }
}

StreamReader::StreamReader(Cam cam) 
{
    this->readImage(cam);
}

void StreamReader::readImage(Cam cam)
{
    std::cout << "Reading images" << std::endl;
    switch(cam){
        case Cam::Cam1:
            readImageFromFolder("/Users/mohit/Code/ComputerVision/VIO/mav0/cam1/", &video);
            break;
        case Cam::Cam0:
        default:
            readImageFromFolder("/Users/mohit/Code/ComputerVision/VIO/mav0/cam0/", &video);
    }
}

void StreamReader::playImageStream() 
{
    std::cout << "Playing video" << std::endl;
    cv::namedWindow("TestWindow", cv::WINDOW_AUTOSIZE);
    for (const auto& img : video) {
        std::cout << "Image size " << img.rows << " x " << img.cols << std::endl;
        cv::imshow("TestWindow", img);
        cv::waitKey(50);
    }
}

void StreamReader::getImage(std::vector<cv::Mat> *outImage) 
{
    outImage = &video;
}
