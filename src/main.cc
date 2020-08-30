#include <iostream>
#include <memory>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include "StreamReader.hpp"
#include "HoughLines.hpp"
#include "Odometry.hpp"

#define PROJ_DIR "/Users/mohit/Code/ComputerVision/VIO/"
#define DATA_DIR "/Users/mohit/Code/ComputerVision/VIO/dataset/MH_01_easy/mav0/"
#define IMG_DIR "../dataset/MH_01_easy/mav0/"

using namespace std;

static std::vector<cv::KeyPoint> getOrbFeatures(const cv::Mat& image) {
    using namespace cv;

    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> extractor = ORB::create();
    vector<KeyPoint> keyPoints;
    Mat descriptor = Mat::zeros(image.size(), CV_8U);
    detector->detectAndCompute(image, Mat(), keyPoints, descriptor);

    return std::move(keyPoints);
}

static void printFeatures(const cv::Mat &img, const std::vector<cv::KeyPoint> &keypoints)
{
    cv::Mat outImg;
    cv::drawKeypoints(img, keypoints, outImg, cv::Scalar(0,0,255));
    cv::imshow("Keypoints", outImg);
    cv::waitKey(50);
}

int main() {
    // initialize odometry parameters.
    Odometry viso(DATA_DIR);

    // read image stream.
    StreamReader videoReader(Cam::Cam0);
    std::shared_ptr<std::vector<cv::Mat>> image = videoReader.getImage();

    // Detect line features (edges)
    // HoughLines lines(1.0, CV_PI/180, 50.0, 50, 10);
    // lines.DetectLines(*image);

    for (const auto& img : *image) {
        // trackedFeatures.track(img);
        std::vector<cv::KeyPoint> orb_features = getOrbFeatures(img);
        printFeatures(img, orb_features);
    }

    return 0;
}
