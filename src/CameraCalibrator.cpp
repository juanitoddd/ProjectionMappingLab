#include "CameraCalibrator.h"
#include "OcvHelpers.h"

void CameraCalibrator::setup(int captureWidth, int captureHeight) {
    gui.setName("Camera Calibration");
    gui.setup();
    gui.add(boardW.setup("Board Width (corners)", 9, 3, 20));
    gui.add(boardH.setup("Board Height (corners)", 6, 3, 20));
    gui.add(squareSz.setup("Square Size (cm)", 2.5f, 1.0f, 10.0f));
    gui.add(btnCapture.setup("Capture Frame (Space)"));
    gui.add(btnCalibrate.setup("Calibrate"));
    gui.add(btnClear.setup("Clear All"));

    btnCapture.addListener(this, &CameraCalibrator::captureFrame);
    btnCalibrate.addListener(this, &CameraCalibrator::calibrate);
    btnClear.addListener(this, &CameraCalibrator::clearFrames);
}

void CameraCalibrator::update(ofPixels& cameraPixels) {
    cv::Mat frame = ofToCv(cameraPixels);
    if (frame.empty()) return;
    cv::cvtColor(frame, lastFrame, cv::COLOR_RGB2GRAY);

    boardSize = cv::Size(boardW, boardH);
    std::vector<cv::Point2f> corners;
    lastFound = cv::findChessboardCorners(lastFrame, boardSize, corners,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);

    if (lastFound) {
        cv::cornerSubPix(lastFrame, corners, cv::Size(11, 11), cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001));
    }

    lastDrawn = frame.clone();
    cv::drawChessboardCorners(lastDrawn, boardSize, corners, lastFound);
}

void CameraCalibrator::drawGui(int x, int y) {
    gui.setPosition(x, y);
    gui.draw();
    ofDrawBitmapStringHighlight("Captured: " + ofToString(getCaptureCount()) + " frames", x, y + gui.getHeight() + 20);
    ofDrawBitmapStringHighlight("Status: " + std::string(calibrated ? "CALIBRATED" : "NOT CALIBRATED"), x, y + gui.getHeight() + 40);
}

void CameraCalibrator::drawPreview(int x, int y, int w, int h) {
    if (lastDrawn.empty()) return;
    drawCvMat(lastDrawn, x, y, w, h);
}

void CameraCalibrator::captureFrame() {
    if (lastFrame.empty() || !lastFound) return;

    std::vector<cv::Point2f> corners;
    bool found = cv::findChessboardCorners(lastFrame, boardSize, corners,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
    if (!found) return;

    cv::cornerSubPix(lastFrame, corners, cv::Size(11, 11), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001));

    imagePoints.push_back(corners);
    buildObjectPoints();
    ofLogNotice("CameraCalibrator") << "Captured frame " << imagePoints.size();
}

bool CameraCalibrator::calibrate() {
    if (imagePoints.size() < 3) {
        ofLogError("CameraCalibrator") << "Need at least 3 frames";
        return false;
    }

    cv::Size imageSize = lastFrame.size();
    std::vector<cv::Mat> rvecs_, tvecs_;
    double rms = cv::calibrateCamera(objectPoints, imagePoints, imageSize,
        cameraMatrix, distCoeffs, rvecs_, tvecs_,
        cv::CALIB_FIX_PRINCIPAL_POINT | cv::CALIB_FIX_ASPECT_RATIO | cv::CALIB_ZERO_TANGENT_DIST);

    rvecs = rvecs_;
    tvecs = tvecs_;
    calibrated = true;
    ofLogNotice("CameraCalibrator") << "Calibrated! RMS=" << rms;
    return true;
}

void CameraCalibrator::clearFrames() {
    imagePoints.clear();
    objectPoints.clear();
    rvecs.clear();
    tvecs.clear();
    calibrated = false;
    ofLogNotice("CameraCalibrator") << "Cleared all frames";
}

void CameraCalibrator::save(const std::string& path, CalibrationData& data) {
    if (!calibrated) return;
    data.cameraMatrix = cameraMatrix.clone();
    data.cameraDistCoeffs = distCoeffs.clone();
    data.cameraImageSize = lastFrame.size();
    data.isCameraCalibrated = true;
    data.save(path);
    ofLogNotice("CameraCalibrator") << "Saved to " << path;
}

void CameraCalibrator::buildObjectPoints() {
    if (objectPoints.size() >= imagePoints.size()) return;
    std::vector<cv::Point3f> pts;
    for (int i = 0; i < boardSize.height; ++i)
        for (int j = 0; j < boardSize.width; ++j)
            pts.emplace_back(j * squareSize, i * squareSize, 0.0f);
    objectPoints.push_back(pts);
}


