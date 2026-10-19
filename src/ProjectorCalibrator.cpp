#include "ProjectorCalibrator.h"
#include "OcvHelpers.h"

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
    gui.add(captureIntervalSlider.setup("Interval (ms)", 500, 100, 2000));
    gui.add(minBrightnessSlider.setup("Min Brightness", 10, 0, 255));
    gui.add(maxBrightnessSlider.setup("Max Brightness", 245, 0, 255));
    gui.add(btnStart.setup("Start New Pose"));
    gui.add(btnPause.setup("Pause"));
    gui.add(btnResume.setup("Resume"));
    gui.add(btnAddPose.setup("Finish Pose (Enter)"));
    gui.add(btnCalibrate.setup("Calibrate"));
    gui.add(btnClear.setup("Clear All"));

    btnStart.addListener(this, &ProjectorCalibrator::startSequence);
    btnPause.addListener(this, &ProjectorCalibrator::onPauseSequence);
    btnResume.addListener(this, &ProjectorCalibrator::onResumeSequence);
    btnAddPose.addListener(this, &ProjectorCalibrator::finishPose);
    btnCalibrate.addListener(this, &ProjectorCalibrator::onCalibrate);
    btnClear.addListener(this, &ProjectorCalibrator::clearPoses);
}

void ProjectorCalibrator::update(ofPixels& cameraPixels) {
    if (blanking && (ofGetElapsedTimef() - blankStartTime < BLANK_DURATION)) {
        patternFbo.begin();
        ofClear(128, 128, 128, 255);
        patternFbo.end();
        return;
    }
    if (blanking) {
        blanking = false;
    }

    if (currentPatternIndex >= 0 && currentPatternIndex < (int)projectedPatterns.size()) {
        patternFbo.begin();
        ofClear(0, 0, 0, 0);
        ofSetColor(255);
        drawCvMat(projectedPatterns[currentPatternIndex], 0, 0);
        patternFbo.end();
        ofLogNotice("ProjectorCalibrator") << "Pattern " << currentPatternIndex;
    } else {
        patternFbo.begin();
        ofClear(128, 128, 128, 255);
        ofSetColor(255);
        if (poses.empty()) {
            ofDrawBitmapString("Projector Calib: Press 'Start New Pose'", 20, 40);
        } else {
            ofDrawBitmapString("Poses captured: " + ofToString((int)poses.size()) + "  Press Start for next", 20, 40);
        }
        patternFbo.end();
    }
}

void ProjectorCalibrator::drawGui(int x, int y) {
    // Dynamically update interval from slider so changes apply to next frame boundary
    if (currentPatternIndex >= 0) {
        captureInterval = captureIntervalSlider / 1000.0f;
    }

    gui.setPosition(x, y);
    gui.draw();
    ofDrawBitmapStringHighlight("Poses: " + ofToString(getPoseCount()), x, y + gui.getHeight() + 20);
    ofDrawBitmapStringHighlight("Current frames: " + ofToString(getCapturedFrameCount()) + " / " + ofToString(projectedPatterns.size()), x, y + gui.getHeight() + 40);
    if (isPaused) {
        ofDrawBitmapStringHighlight("Status: PAUSED", x, y + gui.getHeight() + 60, ofColor(255, 200, 0), ofColor(0));
    } else if (currentPatternIndex >= 0) {
        float cd = getCountdown();
        ofDrawBitmapStringHighlight("Capturing in " + ofToString(cd, 1) + " s", x, y + gui.getHeight() + 60);
    }
    ofDrawBitmapStringHighlight("Status: " + std::string(calibrated ? "CALIBRATED" : "NOT CALIBRATED"), x, y + gui.getHeight() + 80);
}

void ProjectorCalibrator::drawPreview(int x, int y, int w, int h) {
    if (capturedFrames.empty() || capturedFrames.back().empty()) return;
    drawCvMat(capturedFrames.back(), x, y, w, h);
}

void ProjectorCalibrator::startSequence() {
    captureInterval = captureIntervalSlider / 1000.0f; // ms → s
    currentPatternIndex = 0;
    currentRetry = 0;
    isPaused = false;
    blanking = false;
    capturedFrames.clear();
    capturedFrames.resize(projectedPatterns.size());
    lastCaptureTime = ofGetElapsedTimef();
    ofLogNotice("ProjectorCalibrator") << "Started new pose sequence (" << projectedPatterns.size() << " patterns, interval=" << captureInterval << "s)";
}

void ProjectorCalibrator::storeCameraFrame(const cv::Mat& cameraFrame) {
    if (currentPatternIndex < 0 || currentPatternIndex >= (int)projectedPatterns.size() || isPaused) return;

    cv::Mat gray;
    if (cameraFrame.channels() == 3) {
        cv::cvtColor(cameraFrame, gray, cv::COLOR_RGB2GRAY);
    } else if (cameraFrame.channels() == 4) {
        cv::cvtColor(cameraFrame, gray, cv::COLOR_RGBA2GRAY);
    } else {
        gray = cameraFrame.clone();
    }

    if (!validateFrame(gray)) {
        if (currentRetry == 0) {
            currentRetry = 1;
            lastCaptureTime = ofGetElapsedTimef() - captureInterval + captureInterval * 1.5f;
            ofLogWarning("ProjectorCalibrator") << "Frame validation failed for pattern " << currentPatternIndex << ", retrying once...";
            return;
        } else {
            isPaused = true;
            ofLogError("ProjectorCalibrator") << "Frame validation failed again for pattern " << currentPatternIndex << ". Sequence paused.";
            return;
        }
    }

    currentRetry = 0;
    capturedFrames[currentPatternIndex] = gray;
    ofLogNotice("ProjectorCalibrator") << "Stored frame " << currentPatternIndex;

    // Enter blanking period before advancing
    blanking = true;
    blankStartTime = ofGetElapsedTimef();

    currentPatternIndex++;
    markCaptured(); // reset timer for next frame
}

bool ProjectorCalibrator::validateFrame(const cv::Mat& gray) {
    if (gray.empty() || gray.total() == 0) return false;

    double mean = cv::mean(gray)[0];
    float minB = (float)minBrightnessSlider;
    float maxB = (float)maxBrightnessSlider;
    return (mean >= minB && mean <= maxB);
}

void ProjectorCalibrator::finishPose() {
    if (currentPatternIndex < (int)projectedPatterns.size()) {
        ofLogWarning("ProjectorCalibrator") << "Sequence incomplete, cannot finish pose";
        return;
    }

    CalibPose pose;
    pose.frames = capturedFrames;
    decodeCurrentSequence(pose);
    poses.push_back(pose);

    currentPatternIndex = -1;
    capturedFrames.clear();
    ofLogNotice("ProjectorCalibrator") << "Pose finished. Total poses: " << poses.size();
}

void ProjectorCalibrator::clearPoses() {
    poses.clear();
    capturedFrames.clear();
    currentPatternIndex = -1;
    isPaused = false;
    currentRetry = 0;
    blanking = false;
    calibrated = false;
    ofLogNotice("ProjectorCalibrator") << "Cleared all poses";
}

void ProjectorCalibrator::abortSequence() {
    currentPatternIndex = -1;
    isPaused = false;
    currentRetry = 0;
    blanking = false;
    capturedFrames.clear();
    ofLogNotice("ProjectorCalibrator") << "Sequence aborted";
}

void ProjectorCalibrator::onPauseSequence() {
    if (currentPatternIndex < 0 || isPaused) return;
    isPaused = true;
    ofLogNotice("ProjectorCalibrator") << "Sequence paused by user at pattern " << currentPatternIndex;
}

void ProjectorCalibrator::onResumeSequence() {
    if (!isPaused) return;
    isPaused = false;
    currentRetry = 0;
    lastCaptureTime = ofGetElapsedTimef();
    ofLogNotice("ProjectorCalibrator") << "Sequence resumed at pattern " << currentPatternIndex;
}

void ProjectorCalibrator::resumeSequence() {
    // Public resume API (used by ofApp key handler)
    onResumeSequence();
}

bool ProjectorCalibrator::shouldCapture() const {
    if (currentPatternIndex < 0 || currentPatternIndex >= (int)projectedPatterns.size() || isPaused || blanking) return false;
    float elapsed = ofGetElapsedTimef() - lastCaptureTime;
    return elapsed >= captureInterval;
}

void ProjectorCalibrator::markCaptured() {
    lastCaptureTime = ofGetElapsedTimef();
}

float ProjectorCalibrator::getCountdown() const {
    if (currentPatternIndex < 0 || currentPatternIndex >= (int)projectedPatterns.size() || isPaused || blanking) return 0.0f;
    float remaining = captureInterval - (ofGetElapsedTimef() - lastCaptureTime);
    return remaining > 0.0f ? remaining : 0.0f;
}

bool ProjectorCalibrator::calibrate(CalibrationData& cameraData) {
    if (poses.size() < 3) {
        ofLogError("ProjectorCalibrator") << "Need at least 3 poses for stereo calibration";
        return false;
    }
    if (!cameraData.isCameraCalibrated) {
        ofLogError("ProjectorCalibrator") << "Camera must be calibrated first";
        return false;
    }

    std::vector<std::vector<cv::Point3f>> objectPoints;
    std::vector<std::vector<cv::Point2f>> cameraImagePoints;
    std::vector<std::vector<cv::Point2f>> projectorImagePoints;

    for (size_t i = 0; i < poses.size(); ++i) {
        const CalibPose& pose = poses[i];

        // Detect checkerboard in the white frame (last pattern is white)
        const cv::Mat& whiteFrame = pose.frames.back();
        std::vector<cv::Point2f> camCorners;
        if (!extractCheckerboardFromWhiteFrame(whiteFrame, camCorners)) {
            ofLogWarning("ProjectorCalibrator") << "Pose " << i << ": checkerboard not found";
            continue;
        }

        // Build object points (same for each pose)
        std::vector<cv::Point3f> objPts;
        for (int h = 0; h < boardSize.height; ++h) {
            for (int w = 0; w < boardSize.width; ++w) {
                objPts.emplace_back(w * squareSize, h * squareSize, 0.0f);
            }
        }
        objectPoints.push_back(objPts);
        cameraImagePoints.push_back(camCorners);

        // Map each camera corner to projector pixel via decoded correspondences
        std::vector<cv::Point2f> projCorners;
        for (const auto& c : camCorners) {
            int cx = (int)round(c.x);
            int cy = (int)round(c.y);
            if (cx < 0 || cx >= pose.decodedProjX.cols || cy < 0 || cy >= pose.decodedProjX.rows) {
                projCorners.emplace_back(-1, -1);
                continue;
            }
            float px = pose.decodedProjX.at<float>(cy, cx);
            float py = pose.decodedProjY.at<float>(cy, cx);
            float conf = pose.confidence.at<float>(cy, cx);
            if (px < 0 || py < 0 || conf < 0.5f) {
                projCorners.emplace_back(-1, -1);
            } else {
                projCorners.emplace_back(px, py);
            }
        }
        projectorImagePoints.push_back(projCorners);
    }

    if (objectPoints.size() < 2) {
        ofLogError("ProjectorCalibrator") << "Not enough valid poses with checkerboard detection";
        return false;
    }

    // Filter out invalid projector correspondences per pose
    for (size_t i = 0; i < objectPoints.size(); ++i) {
        std::vector<cv::Point3f> validObj;
        std::vector<cv::Point2f> validCam;
        std::vector<cv::Point2f> validProj;
        for (size_t j = 0; j < projectorImagePoints[i].size(); ++j) {
            if (projectorImagePoints[i][j].x >= 0) {
                validObj.push_back(objectPoints[i][j]);
                validCam.push_back(cameraImagePoints[i][j]);
                validProj.push_back(projectorImagePoints[i][j]);
            }
        }
        objectPoints[i] = validObj;
        cameraImagePoints[i] = validCam;
        projectorImagePoints[i] = validProj;
    }

    // Intrinsic initial guesses
    cv::Mat projMatrix = cv::Mat::eye(3, 3, CV_64F);
    projMatrix.at<double>(0, 0) = projW;  // rough fx
    projMatrix.at<double>(1, 1) = projH;  // rough fy
    projMatrix.at<double>(0, 2) = projW * 0.5; // cx
    projMatrix.at<double>(1, 2) = projH * 0.5; // cy
    cv::Mat projDistCoeffs = cv::Mat::zeros(4, 1, CV_64F);

    cv::Mat R, T, E, F;
    cv::TermCriteria criteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 100, 1e-5);

    int flags = cv::CALIB_FIX_INTRINSIC; // keep camera intrinsics fixed, solve projector
    double rms = cv::stereoCalibrate(
        objectPoints,
        cameraImagePoints, projectorImagePoints,
        cameraData.cameraMatrix, cameraData.cameraDistCoeffs,
        projMatrix, projDistCoeffs,
        cv::Size(projW, projH),
        R, T, E, F,
        flags,
        criteria
    );

    ofLogNotice("ProjectorCalibrator") << "Stereo calibration RMS: " << rms;

    // Store in CalibrationData
    // Note: stereoCalibrate returns R, T from camera to projector (or projector to camera?)
    // OpenCV convention: R, T transform points from camera 1 (camera) to camera 2 (projector)
    // So projector_point = R * camera_point + T

    // Save results
    cameraData.projectorMatrix = projMatrix.clone();
    cameraData.projectorDistCoeffs = projDistCoeffs.clone();
    cameraData.projectorImageSize = cv::Size(projW, projH);
    cameraData.isProjectorCalibrated = true;
    cameraData.R = R.clone();
    cameraData.T = T.clone();
    cameraData.isStereoCalibrated = true;
    cameraData.rectify();

    ofLogNotice("ProjectorCalibrator") << "Calibration complete. Results written to passed CalibrationData.";

    calibrated = true;
    return true;
}

void ProjectorCalibrator::save(const std::string& path, CalibrationData& data) {
    if (!calibrated) return;
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

void ProjectorCalibrator::decodeCurrentSequence(CalibPose& pose) {
    if (pose.frames.empty() || pose.frames[0].empty()) return;

    int camW = pose.frames[0].cols;
    int camH = pose.frames[0].rows;
    int bits = grayCodeBits;
    int numStripes = 1 << bits;

    pose.decodedProjX = cv::Mat(camH, camW, CV_32F, cv::Scalar(-1.0f));
    pose.decodedProjY = cv::Mat(camH, camW, CV_32F, cv::Scalar(-1.0f));
    pose.confidence = cv::Mat(camH, camW, CV_32F, cv::Scalar(0.0f));

    // Last two frames are black / white
    int idxBlack = (int)projectedPatterns.size() - 2;
    int idxWhite = (int)projectedPatterns.size() - 1;
    const cv::Mat& black = pose.frames[idxBlack];
    const cv::Mat& white = pose.frames[idxWhite];

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
                float nx = pose.frames[idxXn].at<uchar>(cy, cx);
                float ix = pose.frames[idxXi].at<uchar>(cy, cx);
                float diffX = nx - ix;
                if (std::abs(diffX) < minContrast * 0.3f) { valid = false; break; }
                int bitX = diffX > 0 ? 1 : 0;
                grayX |= (bitX << b);

                int idxYn = (bits * 2) + b * 2;
                int idxYi = (bits * 2) + b * 2 + 1;
                float ny = pose.frames[idxYn].at<uchar>(cy, cx);
                float iy = pose.frames[idxYi].at<uchar>(cy, cx);
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

            pose.decodedProjX.at<float>(cy, cx) = projX;
            pose.decodedProjY.at<float>(cy, cx) = projY;
            pose.confidence.at<float>(cy, cx) = contrast / 255.0f;
        }
    }

    int validPixels = 0;
    for (int y = 0; y < camH; ++y) {
        for (int x = 0; x < camW; ++x) {
            if (pose.decodedProjX.at<float>(y, x) >= 0.0f) validPixels++;
        }
    }
    ofLogNotice("ProjectorCalibrator") << "Decoded pose: " << validPixels << " correspondences";
}

bool ProjectorCalibrator::extractCheckerboardFromWhiteFrame(const cv::Mat& whiteFrame,
                                                            std::vector<cv::Point2f>& corners) {
    bool found = cv::findChessboardCorners(whiteFrame, boardSize, corners,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);
    if (!found) return false;

    cv::cornerSubPix(whiteFrame, corners, cv::Size(11, 11), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001));
    return true;
}

void ProjectorCalibrator::onCalibrate() {
    ofLogNotice("ProjectorCalibrator") << "onCalibrate triggered — call calibrate(cameraData) from ofApp";
}
