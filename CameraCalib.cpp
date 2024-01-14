#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/aruco/aruco_calib.hpp>

#include <format>
#include <string>
#include <iostream>
#include <stdio.h>
#include <filesystem>
#include <vector>

enum class calib_state {
    record,
    calibrate,
    show
};

int main()
{
    cv::VideoCapture inputVideo;
    int deviceID = 0;
    int apiID = cv::CAP_ANY;
    inputVideo.open(deviceID, apiID);
    if (!inputVideo.isOpened()) {
        std::cerr << "ERROR! Unable to open camera" << std::endl;
        return -1;
    }
    float camw = 1280;
    float camh = 720;
    inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, camw);
    inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, camh);
    float aspectRatio = camw / camh;
    calib_state state = calib_state::record;

    cv::Size s{ 7,9 };
    float squareLength = 0.29f;
    float markerLength = 0.21f;
    int calibrationFlags = 0;
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::CharucoBoard board = cv::aruco::CharucoBoard(s, squareLength, markerLength, dictionary );
    cv::aruco::CharucoParameters charucoParams;
    cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
    
    cv::aruco::CharucoDetector detector(board, charucoParams, detectorParams);

    // Collect data from each frame
    std::vector<cv::Mat> allCharucoCorners;
    std::vector<cv::Mat> allCharucoIds;
    std::vector<std::vector<cv::Point2f>> allImagePoints;
    std::vector<std::vector<cv::Point3f>> allObjectPoints;

    cv::Size imageSize;
    cv::Mat image, imageCopy;
    cv::Mat cameraMatrix, distCoeffs;
    while (true) {
        inputVideo.read(image);

        if (image.empty()) {
            std::cerr << "ERROR! blank frame grabbed" << std::endl;
            continue;
        }
        // Draw results
        image.copyTo(imageCopy);
        char key = (char)cv::waitKey(10);
        if (state == calib_state::record) {
            cv::Mat currentCharucoCorners;
            cv::Mat currentCharucoIds;
            std::vector<std::vector<cv::Point2f>> markerCorners, rejectedMarkers;
            std::vector<cv::Point3f> currentObjectPoints;
            std::vector<cv::Point2f> currentImagePoints;
            std::vector<int> markerIds;
                      
            // Detect ChArUco board
            detector.detectBoard(image, currentCharucoCorners, currentCharucoIds, markerCorners, markerIds);
            if (!markerIds.empty()) {
                cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);
            }
            if (currentCharucoIds.total() > 3) {
                cv::aruco::drawDetectedCornersCharuco(imageCopy, currentCharucoCorners, currentCharucoIds);
            }
            if (key == 'c' && currentCharucoCorners.total() > 3) {
                // Match image points
                board.matchImagePoints(currentCharucoCorners, currentCharucoIds, currentObjectPoints, currentImagePoints);

                if (currentImagePoints.empty() || currentObjectPoints.empty()) {
                    std::cout << "Point matching failed, try again." << std::endl;
                    continue;
                }

                std::cout << "Frame captured" << std::endl;

                allCharucoCorners.push_back(currentCharucoCorners);
                allCharucoIds.push_back(currentCharucoIds);
                allImagePoints.push_back(currentImagePoints);
                allObjectPoints.push_back(currentObjectPoints);

                imageSize = image.size();
            }
             cv::imshow("out", imageCopy);
            if (key == 'p') {
                state = calib_state::calibrate;
            }
        }
        else if (state == calib_state::calibrate) {
            double repError = cv::calibrateCamera(
                allObjectPoints, allImagePoints, imageSize,
                cameraMatrix, distCoeffs, cv::noArray(), cv::noArray(), cv::noArray(),
                cv::noArray(), cv::noArray(), calibrationFlags
            );
                       
            state = calib_state::show;
            std::cout<<"Reprojection error: "<<repError<<std::endl;
            std::cout<<"Camera Matrix: "<<cameraMatrix<<std::endl;
            std::cout<<"Distortion Coefficients: "<<distCoeffs<<std::endl;
        }
        else if (state == calib_state::show) {

            cv::Mat currentCharucoIds;
            std::vector<std::vector<cv::Point2f>> markerCorners, rejectedMarkers;
            std::vector<cv::Point3f> currentObjectPoints;
            std::vector<cv::Point2f> currentImagePoints;
            std::vector<int> markerIds;
            cv::Mat undist_image;
            cv::undistort(image, undist_image, cameraMatrix, distCoeffs);

            cv::Mat currentCharucoCorners;
            detector.detectBoard(image, currentCharucoCorners, currentCharucoIds, markerCorners, markerIds);
            if (!markerIds.empty()) {
                // markerCorners, markerIds
                cv::Mat objPoints(4, 1, CV_32FC3);
                objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-markerLength / 2.f, markerLength / 2.f, 0);
                objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerLength / 2.f, markerLength / 2.f, 0);
                objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength / 2.f, -markerLength / 2.f, 0);
                objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-markerLength / 2.f, -markerLength / 2.f, 0);
                int nMarkers = markerCorners.size();
                std::vector<cv::Vec3d> rvecs(nMarkers), tvecs(nMarkers);
                // Calculate pose for each marker
                for (int i = 0; i < nMarkers; i++) {
                    cv::solvePnP(objPoints, markerCorners.at(i), cameraMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
                }
                for (unsigned int i = 0; i < markerIds.size(); i++) {
                    cv::drawFrameAxes(undist_image, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
                }
            }
            cv::imshow("undist_image", undist_image);
        }
       if (key == 27) {
            break;
        }
    }
    inputVideo.release();
    cv::destroyAllWindows();
	return 0;
}