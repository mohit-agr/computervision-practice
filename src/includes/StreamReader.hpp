#ifndef STREAM_READER_H_
#define STREAM_READER_H_

#include <opencv2/core.hpp>

#include <memory>
#include <vector>

enum Cam {
    Cam0, Cam1
};

class StreamReader {
public:
    void readImage(Cam cam);
    void playImageStream();
    std::shared_ptr<std::vector<cv::Mat>> getImage();
    StreamReader(Cam cam);
private:
    std::shared_ptr<std::vector<cv::Mat>> video;
};
#endif // STREAM_READER_H_
