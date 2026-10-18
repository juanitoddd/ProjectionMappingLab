#include "CalibrationData.h"

bool CalibrationData::load(const std::string& path) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) return false;

    fs["cameraMatrix"] >> cameraMatrix;
    fs["cameraDistCoeffs"] >> cameraDistCoeffs;
    fs["projectorMatrix"] >> projectorMatrix;
    fs["projectorDistCoeffs"] >> projectorDistCoeffs;
    fs["R"] >> R;
    fs["T"] >> T;
    fs["P1"] >> P1;
    fs["P2"] >> P2;
    fs["Q"] >> Q;

    fs["cameraImageSize_width"] >> cameraImageSize.width;
    fs["cameraImageSize_height"] >> cameraImageSize.height;
    fs["projectorImageSize_width"] >> projectorImageSize.width;
    fs["projectorImageSize_height"] >> projectorImageSize.height;

    fs["isCameraCalibrated"] >> isCameraCalibrated;
    fs["isProjectorCalibrated"] >> isProjectorCalibrated;
    fs["isStereoCalibrated"] >> isStereoCalibrated;

    return true;
}

bool CalibrationData::save(const std::string& path) const {
    cv::FileStorage fs(path, cv::FileStorage::WRITE);
    if (!fs.isOpened()) return false;

    fs << "cameraMatrix" << cameraMatrix;
    fs << "cameraDistCoeffs" << cameraDistCoeffs;
    fs << "projectorMatrix" << projectorMatrix;
    fs << "projectorDistCoeffs" << projectorDistCoeffs;
    fs << "R" << R;
    fs << "T" << T;
    fs << "P1" << P1;
    fs << "P2" << P2;
    fs << "Q" << Q;

    fs << "cameraImageSize_width" << cameraImageSize.width;
    fs << "cameraImageSize_height" << cameraImageSize.height;
    fs << "projectorImageSize_width" << projectorImageSize.width;
    fs << "projectorImageSize_height" << projectorImageSize.height;

    fs << "isCameraCalibrated" << (int)isCameraCalibrated;
    fs << "isProjectorCalibrated" << (int)isProjectorCalibrated;
    fs << "isStereoCalibrated" << (int)isStereoCalibrated;

    return true;
}

void CalibrationData::rectify() {
    if (!isComplete()) return;

    cv::Mat R1, R2;
    cv::stereoRectify(
        cameraMatrix, cameraDistCoeffs,
        projectorMatrix, projectorDistCoeffs,
        cameraImageSize, R, T,
        R1, R2, P1, P2, Q,
        cv::CALIB_ZERO_DISPARITY, 0,
        cameraImageSize
    );
}
