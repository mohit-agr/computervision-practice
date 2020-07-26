#include "HoughLines.hpp"

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

std::vector<cv::Vec4i> HoughLines::detect_lines(const cv::Mat &image) {
    cv::Mat cannyEdge, dst(image.rows, image.cols, CV_8UC3);
    cv::Canny(image, cannyEdge, 50, 200, 3);

    std::vector<cv::Vec4i> linesP;
    cv::HoughLinesP(cannyEdge, linesP, _rho, _theta, _threshold, _minLineLen, _maxLineGap);

    for(uint i = 0; i < linesP.size(); i++ )
    {
        cv::Vec4i l = linesP[i];
        cv::line(image, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 1, cv::LINE_AA);
    }
    // Show results

    // cv::imshow("source", image);
    // cv::imshow("Lines", dst);
    // cv::waitKey(0);
    return std::move(linesP);
}

void HoughLines::DetectLines(const std::vector<cv::Mat>& imageStream) 
{
    std::cout << "Detecting lines" << std::endl;
    for (const auto& img : imageStream) {
        this->lines.push_back(detect_lines(img));
    }
}

HoughLines::HoughLines(double rho, double theta, double threshold, int min_line_length, int max_line_gap)
{
    this->_rho = rho;
    this->_theta = theta;
    this->_threshold = threshold;
    this->_minLineLen = min_line_length;
    this->_maxLineGap = max_line_gap;
}
