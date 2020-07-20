#include "HoughLines.hpp"

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

std::vector<cv::Vec4i> HoughLines::detect_lines(const cv::Mat &image) {
    cv::Mat cannyEdge, dst;
    cv::Canny(image, cannyEdge, 50, 200, 3);

    std::vector<cv::Vec4i> linesP;
    cv::HoughLinesP(cannyEdge, linesP, 1, CV_PI/180, 50, 50, 10);

    // for( size_t i = 0; i < linesP.size(); i++ )
    // {
    //     cv::Vec4i l = linesP[i];
    //     cv::line( dst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3, cv::LINE_AA);
    // }
    // // Show results
    // imshow("Source", image);
    // imshow("Detected Lines (in red) - Probabilistic Line Transform", dst);

    return std::move(linesP);
}

void HoughLines::DetectLines(const std::vector<cv::Mat>& imageStream) 
{
    for (const auto& img : imageStream) {
        this->lines.push_back(detect_lines(img));
    }
}

HoughLines::HoughLines(double rho, double theta, double threshold, int max_line_length, int max_line_gap) 
{
    this->_rho = rho;
    this->_theta = theta;
    this->_threshold = threshold;
    this->_maxLineLen = max_line_length;
    this->_maxLineGap = max_line_gap;
}
