#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxGui.h"
#include "CalibrationData.h"
#include "CameraCalibrator.h"
#include "ProjectorCalibrator.h"
#include "StructuredLightScanner.h"
#include "MeshViewer.h"
#include "MappingEditor.h"

enum AppState {
    STATE_CAMERA_CALIB,
    STATE_PROJECTOR_CALIB,
    STATE_SCAN,
    STATE_MESH_VIEW,
    STATE_MAP,
    STATE_PERFORM
};

class ofApp : public ofBaseApp {
public:
    void setup() override;
    void update() override;
    void draw() override;
    void drawProjector(ofEventArgs& args);
    void keyPressed(int key) override;
    void keyReleased(int key) override;
    void mouseMoved(int x, int y) override;
    void mouseDragged(int x, int y, int button) override;
    void mousePressed(int x, int y, int button) override;
    void mouseReleased(int x, int y, int button) override;
    void mouseEntered(int x, int y) override;
    void mouseExited(int x, int y) override;
    void windowResized(int w, int h) override;
    void dragEvent(ofDragInfo dragInfo) override;
    void gotMessage(ofMessage msg) override;

    std::shared_ptr<ofAppBaseWindow> projectorWindow;

private:
    AppState state = STATE_CAMERA_CALIB;
    std::string calibPath = "calibration.yml";

    // Hardware
    ofVideoGrabber grabber;
    int camW = 1920, camH = 1080;
    int projW = 1920, projH = 1080;

    // State managers
    CameraCalibrator cameraCalib;
    ProjectorCalibrator projectorCalib;
    StructuredLightScanner scanner;
    MeshViewer meshViewer;
    MappingEditor mappingEditor;
    CalibrationData calibrationData;

    // Projector output
    ofFbo projectorFbo;
    ofCamera projectorCam;

    // GUI
    ofxPanel stateGui;
    ofxButton btnNext, btnPrev;
    ofxLabel stateLabel;

    void setupStateGui();
    void nextState();
    void prevState();
    void updateState();
    void drawState();
    void drawStateGui(int x, int y);
    bool canAdvance() const;
};
