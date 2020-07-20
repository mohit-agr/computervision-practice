#include "StreamReader.hpp"

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#include <fstream>
#include <string>
#include <vector>

static std::shared_ptr<std::vector<cv::Mat>> readImageFromFolder(const std::string &folder)
{
    std::shared_ptr<std::vector<cv::Mat>> imageStream = std::make_shared<std::vector<cv::Mat>>();
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

    return imageStream;
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
            video = readImageFromFolder("/Users/mohit/Code/ComputerVision/VIO/dataset/MH_01_easy/mav0/cam1/");
            break;
        case Cam::Cam0:
        default:
            video = readImageFromFolder("/Users/mohit/Code/ComputerVision/VIO/dataset/MH_01_easy/mav0/cam0/");
    }
}

void StreamReader::playImageStream() 
{
    std::cout << "Playing video" << std::endl;
    cv::namedWindow("TestWindow", cv::WINDOW_AUTOSIZE);
    for (const auto& img : *video) {
        std::cout << "Image size " << img.rows << " x " << img.cols << std::endl;
        cv::imshow("TestWindow", img);
        cv::waitKey(50);
    }
}

std::shared_ptr<std::vector<cv::Mat>> StreamReader::getImage() {
    return video;
}
