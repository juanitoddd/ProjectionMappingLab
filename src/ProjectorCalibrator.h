#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxGui.h"
#include "CalibrationData.h"

struct CalibPose {
    std::vector<cv::Mat> frames;            // captured camera frames for this pose
    cv::Mat decodedProjX;                     // cameraH x cameraW: projector X per camera pixel
    cv::Mat decodedProjY;                     // cameraH x cameraW: projector Y per camera pixel
    cv::Mat confidence;                       // decoding confidence per camera pixel
};

class ProjectorCalibrator {
public:
    void setup(int projectorWidth, int projectorHeight);
    void update(ofPixels& cameraPixels);
    void drawGui(int x, int y);
    void drawPreview(int x, int y, int w, int h);

    // Sequence control
    void startSequence();                     // GUI: begin capturing a new pose
    void storeCameraFrame(const cv::Mat& cameraFrame); // called by ofApp per frame
    void finishPose();                        // commit current captured sequence as a pose
    void clearPoses();
    void abortSequence();
    void resumeSequence();
    bool isSequencePaused() const { return isPaused; }

    // Auto-capture timing
    bool isCapturing() const { return currentPatternIndex >= 0; }
    bool shouldCapture() const;
    void markCaptured();
    float getCountdown() const;

    bool calibrate(CalibrationData& cameraData);
    void onCalibrate();                       // ofxButton wrapper
    void save(const std::string& path, CalibrationData& data);
    bool isCalibrated() const { return calibrated; }
    int getPoseCount() const { return (int)poses.size(); }
    int getCapturedFrameCount() const { return (int)capturedFrames.size(); }
    ofFbo& getPatternFbo() { return patternFbo; }

private:
    int projW, projH;
    bool calibrated = false;
    bool isPaused = false;

    int grayCodeBits = 9;
    int currentPatternIndex = -1; // -1 = idle
    int currentRetry = 0;
    std::vector<cv::Mat> projectedPatterns;

    // Blanking between patterns
    bool blanking = false;
    float blankStartTime = 0.0f;
    static constexpr float BLANK_DURATION = 0.016f;

    // Current sequence being captured
    std::vector<cv::Mat> capturedFrames;

    // Committed poses
    std::vector<CalibPose> poses;

    cv::Size boardSize = cv::Size(9, 6);
    float squareSize = 2.5f;

    ofFbo patternFbo;
    ofxPanel gui;
    ofxIntSlider bitsSlider;
    ofxFloatSlider captureIntervalSlider;
    ofxIntSlider minBrightnessSlider;
    ofxIntSlider maxBrightnessSlider;
    ofxButton btnStart, btnPause, btnResume, btnAddPose, btnCalibrate, btnClear;

    float lastCaptureTime = 0.0f;
    float captureInterval = 0.5f;

    void onPauseSequence();
    void onResumeSequence();
    void generatePatterns();
    cv::Mat grayCodePattern(int axis, int bit, bool inverse);
    void decodeCurrentSequence(CalibPose& pose);
    bool extractCheckerboardFromWhiteFrame(const cv::Mat& whiteFrame,
                                           std::vector<cv::Point2f>& corners);
    bool validateFrame(const cv::Mat& gray);
};
