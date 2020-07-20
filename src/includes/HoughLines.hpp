#ifndef HOUGH_LINES_H_
#define HOUGH_LINES_H_

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#include <memory>
#include <vector>

class HoughLines {
public:
    void DetectLines(const std::vector<cv::Mat>& imageStream);
    HoughLines(double rho, double theta, double threshold, int max_line_length = 30, int max_line_gap = 20);
private:
    std::vector<cv::Vec4i> detect_lines(const cv::Mat &image);
    std::vector<std::vector<cv::Vec4i>> lines;
    double _rho, _theta, _threshold, _maxLineLen, _maxLineGap;
};
#endif // HOUGH_LINES_H_