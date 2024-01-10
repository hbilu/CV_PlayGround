#include <iostream>
#include <opencv2/opencv.hpp>

int main(int argc, char** argv) {
    cv::VideoCapture cap("../Virtual-Paint.mp4");
    if (!cap.isOpened()) {
        std::cerr << "Error: could not open video file" << std::endl;
        return -1;
    }
    cv::namedWindow("Video", 720);

    while (true) {
        cv::Mat frame;
        cap.read(frame);
        if (frame.empty()) {
            break;
        }
        cv::imshow("Video", frame);
        if (cv::waitKey(30) >= 0) {
            break;
        }
    }
    cap.release();
    cv::destroyAllWindows();
    return 0;
}