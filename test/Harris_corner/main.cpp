#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core/core_c.h"

#include <iostream>

using namespace cv;
using namespace std;
Mat src, src_rgb;
int thresh = 150;
int max_thresh = 255;

const char* source_window = "Source image";
const char* corners_window = "Corners detected";

void cornerHarris_demo(int, void*);

int main(int argc, char** argv)
{

    src = imread("../../dataset/MH_01_easy/mav0/cam0/data/1403636579763555584.png", IMREAD_GRAYSCALE); // Read the file
    

    if (!src.data) // Check for invalid input
    {
        cout << "Could not open or find the image" << std::endl;
        return -1;
    }

    src_rgb = cv::Mat(src.size(), CV_8UC3);
    cv::cvtColor(src, src_rgb, COLOR_GRAY2BGR);

    imshow(source_window, src_rgb);
    cornerHarris_demo(0, 0);
    waitKey();
    return 0;
}
void cornerHarris_demo(int, void*)
{
    int blockSize = 2;
    int apertureSize = 3;
    double k = 0.04;
    Mat dst = Mat::zeros(src.size(), CV_8U);
    cornerHarris(src, dst, blockSize, apertureSize, k);
    Mat dst_norm, dst_norm_scaled;
    normalize(dst, dst_norm, 0, 255, NORM_MINMAX, CV_8U, Mat());
    convertScaleAbs(dst_norm, dst_norm_scaled);
    for (int i = 0; i < dst_norm.rows; i++)
    {
        for (int j = 0; j < dst_norm.cols; j++)
        {
            if ((int)dst_norm.at<uint8_t>(i, j) > thresh)
            {
                circle(src_rgb, Point(j, i), 5, Scalar(0,0,255), 2, 8, 0);
            }
        }
    }
    namedWindow(corners_window);
    imshow(corners_window, src_rgb);
}