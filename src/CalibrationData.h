#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"

struct CalibrationData {
    // Camera intrinsics
    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat cameraDistCoeffs = cv::Mat::zeros(4, 1, CV_64F);
    cv::Size cameraImageSize;

    // Projector intrinsics
    cv::Mat projectorMatrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat projectorDistCoeffs = cv::Mat::zeros(4, 1, CV_64F);
    cv::Size projectorImageSize;

    // Stereo rig: camera → projector
    cv::Mat R = cv::Mat::eye(3, 3, CV_64F); // rotation
    cv::Mat T = cv::Mat::zeros(3, 1, CV_64F); // translation

    // Derived (rectified projection matrices for triangulation)
    cv::Mat P1; // camera projection matrix in rectified coord
    cv::Mat P2; // projector projection matrix in rectified coord
    cv::Mat Q;  // disparity-to-depth matrix

    bool isCameraCalibrated = false;
    bool isProjectorCalibrated = false;
    bool isStereoCalibrated = false;

    bool load(const std::string& path);
    bool save(const std::string& path) const;
    bool isComplete() const { return isCameraCalibrated && isProjectorCalibrated && isStereoCalibrated; }
    void rectify();
};
