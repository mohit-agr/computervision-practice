#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include <string>
#include <opencv2/core.hpp>

class Odometry
{
public:
    Odometry(const std::string& filePath);
    Odometry(char* filepath);
private:
    cv::Mat T_LR;
};
#endif // ODOMETRY_H_
