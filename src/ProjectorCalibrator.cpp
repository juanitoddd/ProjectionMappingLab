#include "ProjectorCalibrator.h"
#include "OcvHelpers.h"

// Gray code → binary conversion
static int grayToBinary(int gray) {
    int binary = gray;
    binary ^= (binary >> 1);
    binary ^= (binary >> 2);
    binary ^= (binary >> 4);
    binary ^= (binary >> 8);
    return binary;
}

void ProjectorCalibrator::setup(int projectorWidth, int projectorHeight) {
    projW = projectorWidth;
    projH = projectorHeight;

    patternFbo.allocate(projW, projH, GL_RGB);
    generatePatterns();

    gui.setName("Projector Calibration");
    gui.setup();
    gui.add(bitsSlider.setup("Gray Code Bits", 9, 6, 12));
    gui.add(btnStart.setup("Start Pattern Sequence"));
    gui.add(btnCapture.setup("Capture Frame (Space)"));
    gui.add(btnCalibrate.setup("Calibrate"));
    gui.add(btnClear.setup("Clear All"));

    btnStart.addListener(this, &ProjectorCalibrator::capturePattern);
    btnCapture.addListener(this, &ProjectorCalibrator::capturePattern);
    btnCalibrate.addListener(this, &ProjectorCalibrator::onCalibrate);
    btnClear.addListener(this, &ProjectorCalibrator::clearFrames);
}

void ProjectorCalibrator::update(ofPixels& cameraPixels) {
    if (currentPatternIndex >= 0 && currentPatternIndex < (int)projectedPatterns.size()) {
        patternFbo.begin();
        ofClear(0, 0, 0, 0);
        ofSetColor(255);
        drawCvMat(projectedPatterns[currentPatternIndex], 0, 0);
        patternFbo.end();
        ofLogNotice("ProjectorCalibrator") << "Displaying pattern " << currentPatternIndex;
    } else {
        patternFbo.begin();
        ofClear(128, 128, 128, 255);
        ofSetColor(255);
        ofDrawBitmapString("Projector Calib: Press Start", 20, 40);
        patternFbo.end();
    }
}

void ProjectorCalibrator::drawGui(int x, int y) {
    gui.setPosition(x, y);
    gui.draw();
    ofDrawBitmapStringHighlight("Captured: " + ofToString(getCaptureCount()) + " / " + ofToString(projectedPatterns.size()), x, y + gui.getHeight() + 20);
    ofDrawBitmapStringHighlight("Status: " + std::string(calibrated ? "CALIBRATED" : "NOT CALIBRATED"), x, y + gui.getHeight() + 40);
    if (currentPatternIndex >= 0) {
        ofDrawBitmapStringHighlight("Pattern: " + ofToString(currentPatternIndex), x, y + gui.getHeight() + 60);
    }
}

void ProjectorCalibrator::drawPreview(int x, int y, int w, int h) {
    if (capturedFrames.empty() || capturedFrames.back().empty()) return;
    drawCvMat(capturedFrames.back(), x, y, w, h);
}

void ProjectorCalibrator::capturePattern() {
    // GUI callback – starts/restarts the sequence.
    currentPatternIndex = 0;
    capturedFrames.clear();
    capturedFrames.resize(projectedPatterns.size());
    decodedCameraCoords.release();
    decodedCameraCoordsY.release();
    ofLogNotice("ProjectorCalibrator") << "Started pattern sequence (" << projectedPatterns.size() << " patterns)";
}

void ProjectorCalibrator::storeCameraFrame(const cv::Mat& cameraFrame) {
    if (currentPatternIndex < 0 || currentPatternIndex >= (int)projectedPatterns.size()) return;

    cv::Mat gray;
    if (cameraFrame.channels() == 3) {
        cv::cvtColor(cameraFrame, gray, cv::COLOR_RGB2GRAY);
    } else if (cameraFrame.channels() == 4) {
        cv::cvtColor(cameraFrame, gray, cv::COLOR_RGBA2GRAY);
    } else {
        gray = cameraFrame.clone();
    }

    capturedFrames[currentPatternIndex] = gray;
    ofLogNotice("ProjectorCalibrator") << "Stored frame " << currentPatternIndex;
    currentPatternIndex++;
}

bool ProjectorCalibrator::calibrate(const CalibrationData& cameraData) {
    if (capturedFrames.empty() || capturedFrames[0].empty()) {
        ofLogError("ProjectorCalibrator") << "No captured frames";
        return false;
    }
    if (!cameraData.isCameraCalibrated) {
        ofLogError("ProjectorCalibrator") << "Camera not calibrated";
        return false;
    }

    decodeCorrespondences();

    // We need a planar checkerboard in the scene to establish 3D↔2D correspondences
    // for the projector. This is a placeholder that sets the calibration to true
    // once correspondences are decoded. The actual solvePnP / calibrateCamera call
    // should be performed by ofApp after detecting checkerboard corners in the
    // decoded correspondence map.
    calibrated = true;
    ofLogNotice("ProjectorCalibrator") << "Correspondences decoded. Ready for extrinsics solve (placeholder).";
    return true;
}

void ProjectorCalibrator::clearFrames() {
    capturedFrames.clear();
    currentPatternIndex = -1;
    calibrated = false;
    decodedCameraCoords.release();
    decodedCameraCoordsY.release();
    ofLogNotice("ProjectorCalibrator") << "Cleared";
}

void ProjectorCalibrator::save(const std::string& path, CalibrationData& data) {
    if (!calibrated) return;
    // data.projectorMatrix = ...;
    // data.isProjectorCalibrated = true;
    data.save(path);
    ofLogNotice("ProjectorCalibrator") << "Saved to " << path;
}

void ProjectorCalibrator::generatePatterns() {
    projectedPatterns.clear();
    int bits = grayCodeBits;
    for (int axis = 0; axis < 2; ++axis) {
        int dim = (axis == 0) ? projW : projH;
        for (int b = 0; b < bits; ++b) {
            projectedPatterns.push_back(grayCodePattern(axis, b, false));
            projectedPatterns.push_back(grayCodePattern(axis, b, true));
        }
    }
    // Add all-black and all-white images for photometric normalization
    projectedPatterns.push_back(cv::Mat(projH, projW, CV_8UC1, cv::Scalar(0)));
    projectedPatterns.push_back(cv::Mat(projH, projW, CV_8UC1, cv::Scalar(255)));
}

cv::Mat ProjectorCalibrator::grayCodePattern(int axis, int bit, bool inverse) {
    cv::Mat pat(projH, projW, CV_8UC1);
    int dim = (axis == 0) ? projW : projH;
    int period = dim / (1 << bit);
    if (period < 1) period = 1;

    for (int y = 0; y < projH; ++y) {
        for (int x = 0; x < projW; ++x) {
            int coord = (axis == 0) ? x : y;
            int stripe = (coord / period) & 1;
            if (inverse) stripe = !stripe;
            pat.at<uchar>(y, x) = stripe ? 255 : 0;
        }
    }
    return pat;
}

void ProjectorCalibrator::decodeCorrespondences() {
    if (capturedFrames.empty() || capturedFrames[0].empty()) return;

    int camW = capturedFrames[0].cols;
    int camH = capturedFrames[0].rows;
    int bits = grayCodeBits;
    int numStripes = 1 << bits;

    decodedCameraCoords = cv::Mat(camH, camW, CV_32F, cv::Scalar(-1.0f));
    decodedCameraCoordsY = cv::Mat(camH, camW, CV_32F, cv::Scalar(-1.0f));

    // Last two frames are black / white
    int idxBlack = (int)projectedPatterns.size() - 2;
    int idxWhite = (int)projectedPatterns.size() - 1;
    const cv::Mat& black = capturedFrames[idxBlack];
    const cv::Mat& white = capturedFrames[idxWhite];

    const float minContrast = 15.0f;

    for (int cy = 0; cy < camH; ++cy) {
        for (int cx = 0; cx < camW; ++cx) {
            float b = black.at<uchar>(cy, cx);
            float w = white.at<uchar>(cy, cx);
            float contrast = w - b;
            if (contrast < minContrast) continue;

            int grayX = 0, grayY = 0;
            bool valid = true;

            for (int b = 0; b < bits; ++b) {
                int idxXn = b * 2;
                int idxXi = b * 2 + 1;
                float nx = capturedFrames[idxXn].at<uchar>(cy, cx);
                float ix = capturedFrames[idxXi].at<uchar>(cy, cx);
                float diffX = nx - ix;
                if (std::abs(diffX) < minContrast * 0.3f) { valid = false; break; }
                int bitX = diffX > 0 ? 1 : 0;
                grayX |= (bitX << b);

                int idxYn = (bits * 2) + b * 2;
                int idxYi = (bits * 2) + b * 2 + 1;
                float ny = capturedFrames[idxYn].at<uchar>(cy, cx);
                float iy = capturedFrames[idxYi].at<uchar>(cy, cx);
                float diffY = ny - iy;
                if (std::abs(diffY) < minContrast * 0.3f) { valid = false; break; }
                int bitY = diffY > 0 ? 1 : 0;
                grayY |= (bitY << b);
            }

            if (!valid) continue;

            int stripeX = grayToBinary(grayX);
            int stripeY = grayToBinary(grayY);

            if (stripeX < 0 || stripeX >= numStripes || stripeY < 0 || stripeY >= numStripes) continue;

            float projX = (stripeX + 0.5f) * ((float)projW / numStripes);
            float projY = (stripeY + 0.5f) * ((float)projH / numStripes);

            decodedCameraCoords.at<float>(cy, cx) = projX;
            decodedCameraCoordsY.at<float>(cy, cx) = projY;
        }
    }

    int validPixels = 0;
    for (int y = 0; y < camH; ++y) {
        for (int x = 0; x < camW; ++x) {
            if (decodedCameraCoords.at<float>(y, x) >= 0.0f) validPixels++;
        }
    }
    ofLogNotice("ProjectorCalibrator") << "Decoded " << validPixels << " correspondences";
}

void ProjectorCalibrator::onCalibrate() {
    ofLogNotice("ProjectorCalibrator") << "onCalibrate called – invoke calibrate(cameraData) from ofApp with valid CalibrationData";
}
