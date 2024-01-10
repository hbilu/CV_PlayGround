
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <stdio.h>
#include <iostream>

int main(int argc, char** argv) {
    cv::VideoCapture cap;
    int deviceID = 0;
    int apiID = cv::CAP_ANY;
    cap.open(deviceID, apiID);
    if (!cap.isOpened()) {
        std::cerr << "ERROR! Unable to open camera" << std::endl;
        return -1;
    }
    cv::namedWindow("Video", 720);
    cv::Mat frame;

    while (true) {
        cap.read(frame);
        if (frame.empty()) {
            std::cerr << "ERROR! blank frame grabbed" << std::endl;
            break;
        }
        cv::imshow("Live", frame);
        if (cv::waitKey(30) >= 0) {
            break;
        }
    }
    cap.release();
    cv::destroyAllWindows();
    return 0;
}