#include "StructuredLightScanner.h"
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

void StructuredLightScanner::setup(int projectorWidth, int projectorHeight) {
    projW = projectorWidth;
    projH = projectorHeight;
    patternFbo.allocate(projW, projH, GL_RGB);

    gui.setName("Structured Light Scan");
    gui.setup();
    gui.add(bitsSlider.setup("Gray Code Bits", 9, 6, 12));
    gui.add(btnStart.setup("Start Scan"));
    gui.add(btnCapture.setup("Capture Frame (Space)"));
    gui.add(btnReset.setup("Reset"));

    btnStart.addListener(this, &StructuredLightScanner::startScan);
    btnCapture.addListener(this, &StructuredLightScanner::captureFrame);
    btnReset.addListener(this, &StructuredLightScanner::reset);
}

void StructuredLightScanner::update(ofPixels& cameraPixels) {
    if (scanning && currentPattern < totalPatterns) {
        patternFbo.begin();
        ofClear(0, 0, 0, 0);
        ofSetColor(255);
        if (currentPattern < (int)patterns.size()) {
            drawCvMat(patterns[currentPattern], 0, 0);
        }
        patternFbo.end();
    } else if (!scanning) {
        patternFbo.begin();
        ofClear(64, 64, 64, 255);
        ofSetColor(255);
        ofDrawBitmapString("Ready to scan. Press Start.", 20, 40);
        patternFbo.end();
    }
}

void StructuredLightScanner::drawGui(int x, int y) {
    gui.setPosition(x, y);
    gui.draw();
    ofDrawBitmapStringHighlight("Pattern: " + ofToString(currentPattern) + " / " + ofToString(totalPatterns), x, y + gui.getHeight() + 20);
    ofDrawBitmapStringHighlight("Status: " + std::string(scanComplete ? "COMPLETE" : (scanning ? "SCANNING" : "IDLE")), x, y + gui.getHeight() + 40);
    ofDrawBitmapStringHighlight("Mesh vertices: " + ofToString(mesh.getNumVertices()), x, y + gui.getHeight() + 60);
}

void StructuredLightScanner::drawPreview(int x, int y, int w, int h) {
    if (capturedFrames.empty() || capturedFrames.back().empty()) return;
    drawCvMat(capturedFrames.back(), x, y, w, h);
}

void StructuredLightScanner::startScan() {
    grayCodeBits = bitsSlider;
    generatePatterns();
    scanning = true;
    scanComplete = false;
    currentPattern = 0;
    capturedFrames.clear();
    capturedFrames.resize(totalPatterns);
    decodedCamX.release();
    decodedCamY.release();
    confidence.release();
    ofLogNotice("StructuredLightScanner") << "Scan started, " << totalPatterns << " patterns";
}

void StructuredLightScanner::captureFrame() {
    // This is the GUI callback (button press). ofApp will call storeFrame with the actual camera data.
    ofLogNotice("StructuredLightScanner") << "captureFrame GUI callback – use key or external trigger to supply frame";
}

void StructuredLightScanner::storeFrame(const cv::Mat& cameraFrame) {
    if (!scanning || currentPattern >= totalPatterns) return;

    cv::Mat gray;
    if (cameraFrame.channels() == 3) {
        cv::cvtColor(cameraFrame, gray, cv::COLOR_RGB2GRAY);
    } else if (cameraFrame.channels() == 4) {
        cv::cvtColor(cameraFrame, gray, cv::COLOR_RGBA2GRAY);
    } else {
        gray = cameraFrame.clone();
    }

    capturedFrames[currentPattern] = gray;
    ofLogNotice("StructuredLightScanner") << "Stored frame " << currentPattern << " " << gray.cols << "x" << gray.rows;
    currentPattern++;

    if (currentPattern >= totalPatterns) {
        scanning = false;
        scanComplete = true;
        ofLogNotice("StructuredLightScanner") << "Scan complete, decoding...";
        decode();
    }
}

void StructuredLightScanner::reset() {
    scanning = false;
    scanComplete = false;
    currentPattern = 0;
    capturedFrames.clear();
    patterns.clear();
    hasMesh_ = false;
    mesh.clear();
    decodedCamX.release();
    decodedCamY.release();
    confidence.release();
}

void StructuredLightScanner::generatePatterns() {
    patterns.clear();
    // Gray code: 2 axes × bits × (normal + inverse)
    for (int axis = 0; axis < 2; ++axis) {
        for (int b = 0; b < grayCodeBits; ++b) {
            patterns.push_back(grayCodePattern(axis, b, false));
            patterns.push_back(grayCodePattern(axis, b, true));
        }
    }
    // Phase shift: 2 axes × 3 steps
    for (int axis = 0; axis < 2; ++axis) {
        for (int s = 0; s < 3; ++s) {
            patterns.push_back(phasePattern(axis, s));
        }
    }
    // Black + white frames for thresholding (4 total)
    patterns.push_back(cv::Mat(projH, projW, CV_8UC1, cv::Scalar(0)));
    patterns.push_back(cv::Mat(projH, projW, CV_8UC1, cv::Scalar(255)));
    patterns.push_back(cv::Mat(projH, projW, CV_8UC1, cv::Scalar(0)));
    patterns.push_back(cv::Mat(projH, projW, CV_8UC1, cv::Scalar(255)));

    totalPatterns = (int)patterns.size();
}

cv::Mat StructuredLightScanner::grayCodePattern(int axis, int bit, bool inverse) {
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

cv::Mat StructuredLightScanner::phasePattern(int axis, int step) {
    cv::Mat pat(projH, projW, CV_8UC1);
    float shift = (float)step * (2.0f * PI / 3.0f);
    for (int y = 0; y < projH; ++y) {
        for (int x = 0; x < projW; ++x) {
            float coord = (axis == 0) ? (float)x / projW : (float)y / projH;
            float val = 0.5f + 0.5f * cosf(2.0f * PI * coord * (1 << grayCodeBits) + shift);
            pat.at<uchar>(y, x) = (uchar)(val * 255.0f);
        }
    }
    return pat;
}

void StructuredLightScanner::decode() {
    if (capturedFrames.empty() || capturedFrames[0].empty()) {
        ofLogError("StructuredLightScanner") << "No captured frames to decode";
        return;
    }

    int camW = capturedFrames[0].cols;
    int camH = capturedFrames[0].rows;
    int numStripes = 1 << grayCodeBits;
    float stripeW = (float)projW / numStripes;
    float stripeH = (float)projH / numStripes;

    // Initialize projector-centric maps
    decodedCamX = cv::Mat(projH, projW, CV_32F, cv::Scalar(-1.0f));
    decodedCamY = cv::Mat(projH, projW, CV_32F, cv::Scalar(-1.0f));
    confidence = cv::Mat(projH, projW, CV_32F, cv::Scalar(0.0f));

    // Normalization frames are the last 4
    int idxBlack1 = totalPatterns - 4;
    int idxWhite1 = totalPatterns - 3;
    int idxBlack2 = totalPatterns - 2;
    int idxWhite2 = totalPatterns - 1;

    const cv::Mat& black1 = capturedFrames[idxBlack1];
    const cv::Mat& white1 = capturedFrames[idxWhite1];
    const cv::Mat& black2 = capturedFrames[idxBlack2];
    const cv::Mat& white2 = capturedFrames[idxWhite2];

    // Phase shift frame indices
    int phaseX0 = grayCodeBits * 4;       // 36
    int phaseX1 = phaseX0 + 1;            // 37
    int phaseX2 = phaseX0 + 2;            // 38
    int phaseY0 = phaseX0 + 3;            // 39
    int phaseY1 = phaseX0 + 4;            // 40
    int phaseY2 = phaseX0 + 5;            // 41

    const float minContrast = 15.0f; // camera pixel intensity difference threshold

    for (int cy = 0; cy < camH; ++cy) {
        for (int cx = 0; cx < camW; ++cx) {
            // --- 1. Photometric normalization & confidence ---
            float b1 = black1.at<uchar>(cy, cx);
            float w1 = white1.at<uchar>(cy, cx);
            float b2 = black2.at<uchar>(cy, cx);
            float w2 = white2.at<uchar>(cy, cx);

            float blackAvg = (b1 + b2) * 0.5f;
            float whiteAvg = (w1 + w2) * 0.5f;
            float contrast = whiteAvg - blackAvg;

            if (contrast < minContrast) continue;

            float conf = contrast / 255.0f;
            if (conf > 1.0f) conf = 1.0f;

            // --- 2. Decode Gray code ---
            int grayX = 0, grayY = 0;
            bool valid = true;

            for (int b = 0; b < grayCodeBits; ++b) {
                // X axis frames
                int idxXn = b * 2;
                int idxXi = b * 2 + 1;
                float nx = capturedFrames[idxXn].at<uchar>(cy, cx);
                float ix = capturedFrames[idxXi].at<uchar>(cy, cx);
                float diffX = nx - ix;
                if (std::abs(diffX) < minContrast * 0.3f) { valid = false; break; }
                int bitX = diffX > 0 ? 1 : 0;
                grayX |= (bitX << b);

                // Y axis frames
                int idxYn = (grayCodeBits * 2) + b * 2;
                int idxYi = (grayCodeBits * 2) + b * 2 + 1;
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

            // Clamp to valid range
            if (stripeX < 0 || stripeX >= numStripes || stripeY < 0 || stripeY >= numStripes) continue;

            // --- 3. Phase-shift refinement ---
            float Ix0 = capturedFrames[phaseX0].at<uchar>(cy, cx);
            float Ix1 = capturedFrames[phaseX1].at<uchar>(cy, cx);
            float Ix2 = capturedFrames[phaseX2].at<uchar>(cy, cx);

            float Iy0 = capturedFrames[phaseY0].at<uchar>(cy, cx);
            float Iy1 = capturedFrames[phaseY1].at<uchar>(cy, cx);
            float Iy2 = capturedFrames[phaseY2].at<uchar>(cy, cx);

            // 3-step phase: φ = atan2(√3*(I0-I2), 2*I1 - I0 - I2)
            float phasex = atan2f(sqrtf(3.0f) * (Ix0 - Ix2), 2.0f * Ix1 - Ix0 - Ix2);
            float phasey = atan2f(sqrtf(3.0f) * (Iy0 - Iy2), 2.0f * Iy1 - Iy0 - Iy2);

            // Map [-π, π] → [0, 1]
            float offsetX = (phasex + PI) / (2.0f * PI);
            float offsetY = (phasey + PI) / (2.0f * PI);

            float projXf = (stripeX + offsetX) * stripeW;
            float projYf = (stripeY + offsetY) * stripeH;

            int px = (int)round(projXf);
            int py = (int)round(projYf);

            if (px < 0 || px >= projW || py < 0 || py >= projH) continue;

            // --- 4. Write to projector-centric map (keep highest confidence) ---
            float existingConf = confidence.at<float>(py, px);
            if (conf > existingConf) {
                decodedCamX.at<float>(py, px) = (float)cx;
                decodedCamY.at<float>(py, px) = (float)cy;
                confidence.at<float>(py, px) = conf;
            }
        }
    }

    int validPixels = 0;
    for (int py = 0; py < projH; ++py) {
        for (int px = 0; px < projW; ++px) {
            if (confidence.at<float>(py, px) > 0.0f) validPixels++;
        }
    }

    ofLogNotice("StructuredLightScanner") << "Decoded " << validPixels << " valid projector pixels out of " << (projW * projH);
}

void StructuredLightScanner::triangulate(const CalibrationData& calib) {
    if (!calib.isComplete() || !scanComplete) return;
    if (decodedCamX.empty()) {
        ofLogError("StructuredLightScanner") << "No decoded correspondences";
        return;
    }

    mesh.clear();
    mesh.setMode(OF_PRIMITIVE_TRIANGLES);

    // Camera and projector points for triangulatePoints
    cv::Mat camPoints(1, 1, CV_32FC2);
    cv::Mat projPoints(1, 1, CV_32FC2);

    // For every projector pixel with valid correspondence, triangulate
    for (int py = 0; py < projH; ++py) {
        for (int px = 0; px < projW; ++px) {
            float cx = decodedCamX.at<float>(py, px);
            float cy = decodedCamY.at<float>(py, px);
            float conf = confidence.at<float>(py, px);
            if (conf <= 0.0f || cx < 0 || cy < 0) continue;

            camPoints.at<cv::Point2f>(0, 0) = cv::Point2f(cx, cy);
            projPoints.at<cv::Point2f>(0, 0) = cv::Point2f((float)px, (float)py);

            cv::Mat point4D;
            cv::triangulatePoints(calib.P1, calib.P2, camPoints, projPoints, point4D);

            float w = point4D.at<float>(3, 0);
            if (w == 0) continue;
            glm::vec3 p(
                point4D.at<float>(0, 0) / w,
                point4D.at<float>(1, 0) / w,
                point4D.at<float>(2, 0) / w
            );
            mesh.addVertex(p);
            mesh.addTexCoord(glm::vec2(px / (float)projW, py / (float)projH));
        }
    }

    // Grid triangulation from projector pixel topology
    for (int y = 0; y < projH - 1; ++y) {
        for (int x = 0; x < projW - 1; ++x) {
            int i0 = y * projW + x;
            int i1 = y * projW + (x + 1);
            int i2 = (y + 1) * projW + x;
            int i3 = (y + 1) * projW + (x + 1);

            if (i3 < mesh.getNumVertices()) {
                mesh.addIndex(i0); mesh.addIndex(i1); mesh.addIndex(i2);
                mesh.addIndex(i1); mesh.addIndex(i3); mesh.addIndex(i2);
            }
        }
    }

    hasMesh_ = true;
    ofLogNotice("StructuredLightScanner") << "Mesh reconstructed: " << mesh.getNumVertices() << " vertices, " << (mesh.getNumIndices() / 3) << " triangles";
}
