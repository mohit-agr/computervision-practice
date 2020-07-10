#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"

#include <iostream>
#include <memory>

#include "StreamReader.hpp"

using namespace std;
int main() {
    StreamReader videoReader(Cam::Cam0);
    // videoReader.playImageStream();

    return 0;
}
