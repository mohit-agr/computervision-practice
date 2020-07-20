#include <iostream>
#include <memory>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#include "StreamReader.hpp"
#include "HoughLines.hpp"

using namespace std;

int main() {
    StreamReader videoReader(Cam::Cam0);
    // videoReader.playImageStream();

    std::shared_ptr<std::vector<cv::Mat>> image = videoReader.getImage();

    // Detect line features (edges)
    HoughLines lines(1.0, CV_PI/180, 50.0, 50, 10);
    lines.DetectLines(*image);
    return 0;
}
