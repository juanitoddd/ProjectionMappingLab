#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"

// Helpers to bridge OpenCV cv::Mat and openFrameworks ofPixels/ofTexture
// without requiring the external ofxCv addon.

inline ofPixels cvToOfPixels(const cv::Mat& mat) {
    ofPixels pix;
    if (mat.channels() == 1) {
        pix.allocate(mat.cols, mat.rows, OF_PIXELS_GRAY);
        memcpy(pix.getData(), mat.data, mat.total());
    } else if (mat.channels() == 3) {
        pix.allocate(mat.cols, mat.rows, OF_PIXELS_RGB);
        cv::Mat rgb;
        cv::cvtColor(mat, rgb, cv::COLOR_BGR2RGB);
        memcpy(pix.getData(), rgb.data, mat.total() * 3);
    } else if (mat.channels() == 4) {
        pix.allocate(mat.cols, mat.rows, OF_PIXELS_RGBA);
        cv::Mat rgba;
        cv::cvtColor(mat, rgba, cv::COLOR_BGRA2RGBA);
        memcpy(pix.getData(), rgba.data, mat.total() * 4);
    }
    return pix;
}

inline cv::Mat ofToCv(const ofPixels& pix) {
    int type = CV_8UC1;
    if (pix.getNumChannels() == 3) type = CV_8UC3;
    else if (pix.getNumChannels() == 4) type = CV_8UC4;
    return cv::Mat(pix.getHeight(), pix.getWidth(), type, const_cast<unsigned char*>(pix.getData()));
}

inline void drawCvMat(const cv::Mat& mat, float x, float y, float w = 0, float h = 0) {
    static ofTexture tex;
    if (mat.empty()) return;
    ofPixels pix = cvToOfPixels(mat);
    tex.loadData(pix);
    if (w <= 0 || h <= 0) {
        tex.draw(x, y);
    } else {
        tex.draw(x, y, w, h);
    }
}
