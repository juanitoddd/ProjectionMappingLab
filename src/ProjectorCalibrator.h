#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxGui.h"
#include "CalibrationData.h"

class ProjectorCalibrator {
public:
    void setup(int projectorWidth, int projectorHeight);
    void update(ofPixels& cameraPixels);
    void drawGui(int x, int y);
    void drawPreview(int x, int y, int w, int h);
    void capturePattern();              // GUI callback: starts sequence
    void storeCameraFrame(const cv::Mat& cameraFrame); // called by ofApp
    bool calibrate(const CalibrationData& cameraData);
    void onCalibrate();                 // wrapper for GUI callback
    void clearFrames();
    void save(const std::string& path, CalibrationData& data);
    bool isCalibrated() const { return calibrated; }
    int getCaptureCount() const { return (int)capturedFrames.size(); }
    ofFbo& getPatternFbo() { return patternFbo; }

private:
    int projW, projH;
    bool calibrated = false;

    // Gray code projector patterns for this calibration step
    int grayCodeBits = 9;
    int currentPatternIndex = -1; // -1 = not projecting patterns
    std::vector<cv::Mat> capturedFrames; // camera views of projected patterns
    std::vector<cv::Mat> projectedPatterns; // binary patterns we sent

    cv::Mat decodedCameraCoords; // per projector pixel, which camera pixel saw it
    cv::Mat decodedCameraCoordsY;
    std::vector<cv::Point3f> worldPoints; // 3D points on calibration plane
    std::vector<cv::Point2f> projectorPoints; // corresponding projector pixels

    ofFbo patternFbo;
    ofxPanel gui;
    ofxIntSlider bitsSlider;
    ofxButton btnStart, btnCapture, btnCalibrate, btnClear;

    void generatePatterns();
    void decodeCorrespondences();
    cv::Mat grayCodePattern(int axis, int bit, bool inverse);
};
