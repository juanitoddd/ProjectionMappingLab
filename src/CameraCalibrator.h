#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxGui.h"
#include "CalibrationData.h"

class CameraCalibrator {
public:
    void setup(int captureWidth, int captureHeight);
    void update(ofPixels& cameraPixels);
    void drawGui(int x, int y);
    void drawPreview(int x, int y, int w, int h);
    void captureFrame();
    bool calibrate();
    void clearFrames();
    void save(const std::string& path, CalibrationData& data);
    bool isCalibrated() const { return calibrated; }
    int getCaptureCount() const { return (int)imagePoints.size(); }

private:
    cv::Size boardSize = cv::Size(9, 6); // internal corners
    float squareSize = 2.5f; // cm (only for scale, arbitrary works for intrinsics)
    bool calibrated = false;

    std::vector<std::vector<cv::Point2f>> imagePoints;
    std::vector<std::vector<cv::Point3f>> objectPoints;
    std::vector<cv::Mat> rvecs, tvecs;

    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64F);
    cv::Mat lastFrame;
    cv::Mat lastDrawn;
    bool lastFound = false;

    ofxPanel gui;
    ofxIntSlider boardW, boardH;
    ofxFloatSlider squareSz;
    ofxButton btnCapture, btnCalibrate, btnClear;

    void buildObjectPoints();
};
