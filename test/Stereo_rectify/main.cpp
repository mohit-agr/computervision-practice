#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core/core_c.h"
#include <opencv2/calib3d.hpp>
#include "opencv2/highgui.hpp"
#include <iostream>

#define PRINT_DEBUG

using namespace std;

cv::Mat img_left, img_right;
cv::Mat img_left_rect, img_right_rect;
cv::Mat disparity_map, disparity_map_8U;

const char* source_window = "Source image";
static void saveXYZ(const char* filename, const cv::Mat& mat);
int main(int argc, char** argv)
{

    img_left = cv::imread("../../dataset/cam_checkerboard/mav0/cam0/data/1403709075387836928.png", cv::IMREAD_GRAYSCALE); // Read the file
    img_right = cv::imread("../../dataset/cam_checkerboard/mav0/cam1/data/1403709075387836928.png", cv::IMREAD_GRAYSCALE); // Read the file

    if (!img_left.data || !img_right.data) // Check for invalid input
    {
        cout << "Could not open or find the image" << std::endl;
        return -1;
    }

     // EuROC Camera Extrinsic parameters with respect to the base frame. Left is cam0. Right is cam1.
    cv::Mat Camera_Extrinsic_Left = (cv::Mat1d(4, 4) << 0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975, \
        0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768, \
        - 0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949, \
        0.0, 0.0, 0.0, 1.0);
    cv::Mat Camera_Extrinsic_Right = (cv::Mat1d(4, 4) << 0.0125552670891, -0.999755099723, 0.0182237714554, -0.0198435579556, \
        0.999598781151, 0.0130119051815, 0.0251588363115, 0.0453689425024, \
        - 0.0253898008918, 0.0179005838253, 0.999517347078, 0.00786212447038, \
        0.0, 0.0, 0.0, 1.0);
#ifdef PRINT_DEBUG
    std::cout << "Extrinsic Left = " << Camera_Extrinsic_Left << std::endl;
    std::cout << "Extrinsic Right = " << Camera_Extrinsic_Right << std::endl;
#endif

    cv::Mat Camera_Extrinsic = (Camera_Extrinsic_Left.inv()*Camera_Extrinsic_Right).inv();
#ifdef PRINT_DEBUG
    std::cout << "Extrinsic  = " << Camera_Extrinsic << std::endl;
#endif

    cv::Mat Camera_R = Camera_Extrinsic(cv::Rect(0, 0, 3, 3));
    cv::Mat Camera_T = Camera_Extrinsic(cv::Rect(3, 0, 1, 3));

#ifdef PRINT_DEBUG
    std::cout << "Extrinsic Rotation = " << Camera_R << std::endl;
    std::cout << "Extrinsic Translation = " << Camera_T << std::endl;
#endif

    // Camera matrix (Intrinsic parameters)
    cv::Mat Camera_Matrix_Left = (cv::Mat1d(3, 3) << 458.654, 0, 367.215, 0, 457.296, 248.375, 0.0, 0.0, 1.0);
    cv::Mat Camera_Matrix_Right = (cv::Mat1d(3, 3) << 457.587, 0, 379.999, 0, 456.134, 255.238, 0.0, 0.0, 1.0);

    #ifdef PRINT_DEBUG
        std::cout << "Camera Matrix left = " << Camera_Matrix_Left << std::endl;
        std::cout << "Camera Matrix right = " << Camera_Matrix_Right << std::endl;
    #endif

    // distortion coefficient
    cv::Mat distortion_coeff_left = (cv::Mat1d(1, 5) << -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05, 0);
    cv::Mat distortion_coeff_right = (cv::Mat1d(1, 5) << -0.28368365, 0.07451284, -0.00010473, -3.55590700e-05, 0);

    #ifdef PRINT_DEBUG
        std::cout << "Distortion left = " << distortion_coeff_left << std::endl;
        std::cout << "Distortion right = " << distortion_coeff_right << std::endl;
    #endif

    // stereo rectify coefficients
    cv::Mat R1 = cv::Mat1d(3, 3);
    cv::Mat R2 = cv::Mat1d(3, 3);
    cv::Mat P1 = cv::Mat1d(3, 4);
    cv::Mat P2 = cv::Mat1d(3, 4);
    cv::Mat Q = cv::Mat1d(4, 4);
    cv::Rect validRoi[2];

    int rect_alpha = 0; // alpha=1 for the whole image, but if there is too much warp, make it less.
    cv::stereoRectify(Camera_Matrix_Left, distortion_coeff_left, Camera_Matrix_Right, \
        distortion_coeff_right, img_left.size(), Camera_R, Camera_T, R1, R2, P1, P2, \
        Q, 0, rect_alpha, img_left.size(), &validRoi[0]);

#ifdef PRINT_DEBUG
    std::cout << "R1" << R1 << std::endl;
    std::cout << "R2" << R2 << std::endl;
    std::cout << "P1" << P1 << std::endl;
    std::cout << "P2" << P2 << std::endl;
    std::cout << "Q" << Q << std::endl;
#endif

/*    // Taken from ORB_SLAM2 git repo https://github.com/raulmur/ORB_SLAM2/blob/master/Examples/Stereo/EuRoC.yaml
    R1 = (cv::Mat1d(3,3) << 0.999966347530033, -0.001422739138722922, 0.008079580483432283, 0.001365741834644127, 0.9999741760894847, 
                            0.007055629199258132, -0.008089410156878961, -0.007044357138835809, 0.9999424675829176);
    R2 = (cv::Mat1d(3, 3) << 0.9999633526194376, -0.003625811871560086, 0.007755443660172947, 0.003680398547259526, 0.9999684752771629,
                            -0.007035845251224894, -0.007729688520722713, 0.007064130529506649, 0.999945173484644);
    P1 = (cv::Mat1d(3, 4) << 435.2046959714599, 0, 367.4517211914062, 0, 0, 435.2046959714599, 252.2008514404297, 0, 0, 0, 1, 0);
    P2 = (cv::Mat1d(3, 4) << 435.2046959714599, 0, 367.4517211914062, -47.90639384423901, 0, 435.2046959714599, 252.2008514404297, 0, 0, 0, 1, 0);
*/    
    // rectify images
    cv::Mat map11, map12, map21, map22;
    cv::initUndistortRectifyMap(Camera_Matrix_Left, distortion_coeff_left, R1, P1, img_left.size(), CV_16SC2, map11, map12);
    cv::initUndistortRectifyMap(Camera_Matrix_Right, distortion_coeff_right, R2, P2, img_right.size(), CV_16SC2, map21, map22);
    cv::remap(img_left, img_left_rect, map11, map12, cv::INTER_LINEAR);
    cv::remap(img_right, img_right_rect, map21, map22, cv::INTER_LINEAR);

    img_left = img_left_rect;
    img_right = img_right_rect;

    cv::Mat img_concat, img_concat_color;
    cv::hconcat(img_left, img_right, img_concat);
    cv::cvtColor(img_concat, img_concat_color, cv::COLOR_GRAY2RGB);

    for (int j = 0; j < img_concat_color.rows; j += 16)
       cv::line(img_concat_color, cv::Point(0, j), cv::Point(img_concat.cols, j), cv::Scalar(0, 255, 0), 1, 8);

    //cv::Mat image3DOCV;
    //cv::reprojectImageTo3D(depth_map_normalized, image3DOCV, Q, false, CV_32F);

    cv::imshow("Rectified stereo images", img_concat_color);
    cv::waitKey(0);

    
    // SGBM parameters
    int windowSize = 9;
    int minDisparity = 16;
    int numDisparity = 112 - minDisparity;
    int maxBlockSize = 11;
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(minDisparity, numDisparity, 16, \
        8 * windowSize * windowSize, 32 * windowSize * windowSize, \
        1, 0, 10, 160, 32, cv::StereoSGBM::MODE_SGBM);

    #ifdef PRINT_DEBUG
        std::cout << "Calculating disparity map..." << std::endl;
    #endif

    sgbm->compute(img_left, img_right, disparity_map);
    // Normalize image 0 to 255.
    // imshow can display only CV_8U!
    disparity_map.convertTo(disparity_map_8U, CV_8U, 255 / (16.0 * sgbm->getNumDisparities()), \
        - sgbm->getMinDisparity() / sgbm->getNumDisparities());

    cv::imshow("Disparity", disparity_map_8U);
    cv::waitKey(0);
    #ifdef PRINT_DEBUG
        std::cout << "Disparity map calculation completed." << std::endl;
        std::cout << "Generating Point Cloud" << std::endl;
    #endif

    cv::Mat xyz;
    cv::Mat pclTemp;
    disparity_map.convertTo(pclTemp, CV_32F, 1.0 / 16);
    cv::reprojectImageTo3D(pclTemp, xyz, Q, true);
    saveXYZ("PCL_test.pcl", xyz);
    return 0;
}

static void saveXYZ(const char* filename, const cv::Mat& mat)
{
    const double max_z = 1.0e4;
    FILE* fp = fopen(filename, "wt");
    for (int y = 0; y < mat.rows; y++)
    {
        for (int x = 0; x < mat.cols; x++)
        {
            cv::Vec3f point = mat.at<cv::Vec3f>(y, x);
            if (fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
            fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);
        }
    }
    fclose(fp);
}

