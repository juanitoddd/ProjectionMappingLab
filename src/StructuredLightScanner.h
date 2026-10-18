#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxGui.h"
#include "CalibrationData.h"

class StructuredLightScanner {
public:
    void setup(int projectorWidth, int projectorHeight);
    void update(ofPixels& cameraPixels);
    void drawGui(int x, int y);
    void drawPreview(int x, int y, int w, int h);
    void startScan();
    void captureFrame(); // GUI callback only
    void storeFrame(const cv::Mat& cameraFrame); // called by ofApp with live frame
    bool isScanning() const { return scanning; }
    bool isComplete() const { return scanComplete; }
    void reset();

    // Access decoded data
    cv::Mat& getCameraX() { return decodedCamX; }
    cv::Mat& getCameraY() { return decodedCamY; }
    cv::Mat& getConfidence() { return confidence; }

    // Reconstruct 3D mesh from correspondences + calibration
    void triangulate(const CalibrationData& calib);
    ofVboMesh& getMesh() { return mesh; }
    bool hasMesh() const { return hasMesh_; }

    ofFbo patternFbo;

private:
    int projW, projH;
    bool scanning = false;
    bool scanComplete = false;
    int currentPattern = 0;
    int totalPatterns = 0;

    // Pattern sequence
    int grayCodeBits = 9;
    std::vector<cv::Mat> patterns;
    std::vector<cv::Mat> capturedFrames;

    // Decoded correspondences: per projector pixel, which camera pixel saw it?
    cv::Mat decodedCamX, decodedCamY, confidence;

    // 3D mesh
    ofVboMesh mesh;
    bool hasMesh_ = false;
    ofxPanel gui;
    ofxIntSlider bitsSlider;
    ofxButton btnStart, btnCapture, btnReset;

    void generatePatterns();
    cv::Mat grayCodePattern(int axis, int bit, bool inverse);
    cv::Mat phasePattern(int axis, int step);
    void decode();
};
