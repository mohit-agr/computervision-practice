#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core/core_c.h"
#include <opencv2/calib3d.hpp>
#include "opencv2/highgui.hpp"
#include <iostream>

//#define PRINT_DEBUG

using namespace std;

cv::Mat img_left, img_right, img_left_scale, img_right_scale;
cv::Mat img_left_rect, img_right_rect;
cv::Mat disparity_map, disparity_map_8U;

// SGBM parameters
int windowSize = 3;
int minDisparity = 16;
int numDisparity = 112 - minDisparity;
int maxBlockSize = 11;

const char* source_window = "Source image";
void demo_gui(int, void*);

int main(int argc, char** argv)
{

    //img_left = cv::imread("../../dataset/MH_01_easy/mav0/cam0/data/1403636705813555456.png", cv::IMREAD_GRAYSCALE); // Read the file
    //img_right = cv::imread("../../dataset/MH_01_easy/mav0/cam1/data/1403636705813555456.png", cv::IMREAD_GRAYSCALE); // Read the file
    img_left = cv::imread("testset/teddy/imL.png", cv::IMREAD_GRAYSCALE); // Read the file
    img_right = cv::imread("testset/teddy/imR.png", cv::IMREAD_GRAYSCALE); // Read the file

    if (!img_left.data || !img_right.data) // Check for invalid input
    {
        cout << "Could not open or find the image" << std::endl;
        return -1;
    }

    // scale the image for faster processing
    float scale = 1;
    cv::resize(img_left, img_left_scale, cv::Size(), scale, scale, cv::INTER_LINEAR);
    cv::resize(img_right, img_right_scale, cv::Size(), scale, scale, cv::INTER_LINEAR);
   
    // EuROC Camera Extrinsic parameters with respect to the base frame. Left is cam0. Right is cam1.
    cv::Mat Camera_Extrinsic_Left = (cv::Mat1d(4,4) << 0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975, \
                                                       0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768, \
                                                       -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949, \
                                                       0.0, 0.0, 0.0, 1.0);
    cv::Mat Camera_Extrinsic_Right = (cv::Mat1d(4, 4) << 0.0125552670891, -0.999755099723, 0.0182237714554, -0.0198435579556, \
                                                         0.999598781151, 0.0130119051815, 0.0251588363115, 0.0453689425024, \
                                                         -0.0253898008918, 0.0179005838253, 0.999517347078, 0.00786212447038, \
                                                         0.0, 0.0, 0.0, 1.0);
    #ifdef PRINT_DEBUG
        std::cout << "Extrinsic Left = " << Camera_Extrinsic_Left << std::endl;
        std::cout << "Extrinsic Right = " << Camera_Extrinsic_Right << std::endl;
    #endif

    cv::Mat Camera_Extrinsic = Camera_Extrinsic_Right.inv() * Camera_Extrinsic_Left;
    #ifdef PRINT_DEBUG
        std::cout << "Extrinsic  = " << Camera_Extrinsic << std::endl;
    #endif

    cv::Mat Camera_R = Camera_Extrinsic(cv::Rect(0,0,3,3));
    cv::Mat Camera_T = Camera_Extrinsic(cv::Rect(3, 0, 1, 3));

    #ifdef PRINT_DEBUG
        std::cout << "Extrinsic Rotation = " << Camera_R << std::endl;
        std::cout << "Extrinsic Translation = " << Camera_T << std::endl;
    #endif

    // Camera matrix (Intrinsic parameters)
    cv::Mat Camera_Matrix_Left = (cv::Mat1d(3,3) << 458.654, 0, 367.215, 0, 457.296, 248.375, 0, 0, 1);
    cv::Mat Camera_Matrix_Right = (cv::Mat1d(3, 3) << 457.587, 0, 379.999, 0, 456.134, 255.238, 0, 0, 1);

    // distortion coefficient
    cv::Mat distortion_coeff_left = (cv::Mat1d(1,5) << -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05, 0);
    cv::Mat distortion_coeff_right = (cv::Mat1d(1, 5) << -0.28368365, 0.07451284, -0.00010473, -3.55590700e-05, 0);

    // stereo rectify coefficients
    cv::Mat R1 = cv::Mat1d(3,3);
    cv::Mat R2 = cv::Mat1d(3, 3);
    cv::Mat P1 = cv::Mat1d(3, 4);
    cv::Mat P2 = cv::Mat1d(3, 4);
    cv::Mat Q = cv::Mat1d(4, 4);
    cv::Rect validRoi[2];

    cv::stereoRectify(Camera_Matrix_Left, distortion_coeff_left, Camera_Matrix_Right,\
                        distortion_coeff_right, img_left.size(), Camera_R, Camera_T, R1, R2, P1, P2, \
                         Q, 0, 1, img_left.size(), &validRoi[0]);

    #ifdef PRINT_DEBUG
        std::cout << "R1" << R1 << std::endl;
        std::cout << "R2" << R2 << std::endl;
        std::cout << "P1" << P1 << std::endl;
        std::cout << "P2" << P2 << std::endl;
        std::cout << "Q" << Q << std::endl;
    #endif

    // rectify images
    cv::Mat map11, map12, map21, map22;
    cv::initUndistortRectifyMap(Camera_Matrix_Left, distortion_coeff_left, R1, P1, img_left.size(), CV_16SC2, map11, map12);
    cv::initUndistortRectifyMap(Camera_Matrix_Right, distortion_coeff_right, R1, P1, img_right.size(), CV_16SC2, map21, map22);
    cv::remap(img_left, img_left_rect, map11, map12, cv::INTER_LINEAR);
    cv::remap(img_right, img_right_rect, map21, map22, cv::INTER_LINEAR);

    img_left = img_left_rect;
    img_right = img_right_rect;

    //cv::Mat image3DOCV;
    //cv::reprojectImageTo3D(depth_map_normalized, image3DOCV, Q, false, CV_32F);

    cv::namedWindow(source_window);
    cv::createTrackbar("Block Size: ", source_window, &windowSize, maxBlockSize, demo_gui);
    demo_gui(0, 0);
    cv::waitKey(0);
    return 0;
}

void demo_gui(int, void*)
{   // block size should be odd and in between 3 to 11.
    if (windowSize < 3 || windowSize > 11 || windowSize % 2 != 1) {
        return;
    }

    #ifdef PRINT_DEBUG
        std::cout << "Block Size = " << windowSize << std::endl;
    #endif

    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(minDisparity, numDisparity, 16, \
                                    8 * windowSize * windowSize, 32 * windowSize * windowSize, \
                                    1, 0, 10, 100, 32, cv::StereoSGBM::MODE_SGBM);
    #ifdef PRINT_DEBUG
        std::cout << "Calculating disparity map..." << std::endl;
    #endif

    sgbm->compute(img_left_scale, img_right_scale, disparity_map);

    #ifdef PRINT_DEBUG
        double min, max;
        cv::minMaxLoc(disparity_map, &min, &max);
        std::cout << "Disparity map Minimum = " << min << std::endl;
        std::cout << "Disparity map Maximum = " << max << std::endl;
    #endif

    // Normalize image 0 to 255.
    // imshow can display only CV_8U!
    disparity_map.convertTo(disparity_map_8U, CV_8U, 255/(16.0 * sgbm->getNumDisparities()), \
                            -sgbm->getMinDisparity()/ sgbm->getNumDisparities());

    #ifdef PRINT_DEBUG
        cv::minMaxLoc(disparity_map_8U, &min, &max);
        std::cout << "Scaled Disparity map Minimum = " << min << std::endl;
        std::cout << "Scaled Disparity map Maximum = " << max << std::endl;
    #endif

    cv::imshow(source_window, disparity_map_8U);

    #ifdef PRINT_DEBUG
        std::cout << "Disparity map calculation completed." << std::endl;
    #endif

}