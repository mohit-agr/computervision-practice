#include "Odometry.hpp"

#include <iostream>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

using namespace cv;
using namespace std;

static void readTransformation(const FileStorage& fs, cv::Mat *T) {
    if (!fs.isOpened())
    {
        cerr << "Failed to open " << endl;
        return;
    }

    FileNode n = fs["T_BS"];                         // Read string sequence - Get node
    n >> (*T);
}

Odometry::Odometry(const std::string& filePath) 
{
    FileStorage left, right;
    std::string leftFile = filePath + std::string("cam0/sensor.yaml");
    std::string rightFile = filePath + std::string("cam1/sensor.yaml");
    left.open(leftFile, FileStorage::READ);
    right.open(rightFile, FileStorage::READ);

    cv::Mat T_LB, T_RB, T_BR;
    readTransformation(left, &T_LB);
    readTransformation(right, &T_RB);

    cv::invert(T_RB, T_BR);
    cv::multiply(T_LB, T_BR, this->T_LR);
}

Odometry::Odometry(char* filepath) 
{
    Odometry(std::string(filepath));
}
