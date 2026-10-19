#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxGui.h"
#include "CalibrationData.h"

struct VerificationResult {
    float rmsError = -1.0f;          // RMS reprojection error in projector pixels
    int numCorrespondences = 0;       // number of valid corner correspondences used
    bool passed = false;              // true if rmsError < threshold
    std::vector<cv::Point2f> cameraCorners;
    std::vector<cv::Point2f> observedProjCorners;
    std::vector<cv::Point2f> predictedProjCorners;
};

class CalibrationVerifier {
public:
    void setup(int boardW, int boardH, float squareSize);
    void update(ofPixels& cameraPixels);
    void drawGui(int x, int y);
    void drawPreview(int x, int y, int w, int h);

    void captureVerifyPose(const CalibrationData& calib);
    void computeReprojectionError(const CalibrationData& calib);
    void applyFineTuningOffsets(CalibrationData& calib);
    void resetOffsets();

    const VerificationResult& getResult() const { return result; }
    bool hasResult() const { return result.rmsError >= 0.0f; }

    // Fine-tuning sliders exposed for external GUI if desired
    ofxFloatSlider offsetYaw, offsetPitch, offsetRoll;
    ofxFloatSlider offsetTx, offsetTy, offsetTz;

private:
    cv::Size boardSize;
    float squareSize;
    cv::Mat lastFrame;
    cv::Mat lastDrawn;
    bool lastFound = false;

    ofxPanel gui;
    ofxButton btnCapture, btnCompute, btnApply, btnResetOffsets;
    ofxFloatSlider passThreshold;

    VerificationResult result;
    cv::Mat capturedFrame;

    void onCapture();
    void onCompute();
    void onApply();
    void onReset();
    cv::Mat buildAdjustedExtrinsics(const CalibrationData& calib);
};
