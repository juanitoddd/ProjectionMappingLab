#include "ofApp.h"
#include "OcvHelpers.h"

//--------------------------------------------------------------
void ofApp::setup() {
    ofSetVerticalSync(true);
    ofSetFrameRate(60);
    ofSetWindowTitle("ProjectionMappingLab - GUI");

    // Setup camera
    grabber.setDeviceID(0);
    grabber.setDesiredFrameRate(30);
    grabber.setup(camW, camH);
    camW = grabber.getWidth();
    camH = grabber.getHeight();

    // Setup projector FBO
    projectorFbo.allocate(projW, projH, GL_RGBA);

    // Setup managers
    cameraCalib.setup(camW, camH);
    projectorCalib.setup(projW, projH);
    scanner.setup(projW, projH);
    meshViewer.setup();
    mappingEditor.setup(projW, projH);

    // Load existing calibration
    if (calibrationData.load(calibPath)) {
        ofLogNotice("ofApp") << "Loaded existing calibration from " << calibPath;
    }

    setupStateGui();

    // Register projector draw callback
    if (projectorWindow) {
        ofAddListener(projectorWindow->events().draw, this, &ofApp::drawProjector);
    }
}

void ofApp::setupStateGui() {
    stateGui.setup();
    stateGui.setName("State Control");
    stateGui.add(stateLabel.setup("State", "CAMERA_CALIB"));
    stateGui.add(btnPrev.setup("<< Prev"));
    stateGui.add(btnNext.setup("Next >>"));
    btnNext.addListener(this, &ofApp::nextState);
    btnPrev.addListener(this, &ofApp::prevState);
}

//--------------------------------------------------------------
void ofApp::update() {
    grabber.update();

    if (grabber.isFrameNew()) {
        ofPixels& px = grabber.getPixels();
        switch (state) {
        case STATE_CAMERA_CALIB:
            cameraCalib.update(px);
            break;
        case STATE_PROJECTOR_CALIB:
            projectorCalib.update(px);
            break;
        case STATE_SCAN:
            scanner.update(px);
            break;
        default:
            break;
        }
    }

    // Update managers
    meshViewer.update();
    mappingEditor.update(ofGetElapsedTimef());

    // Auto-capture: feed camera frames to active sequences at timed intervals
    if (state == STATE_SCAN && scanner.isScanning()) {
        if (grabber.isFrameNew() && scanner.shouldCapture()) {
            cv::Mat cam = ofToCv(grabber.getPixels());
            cv::Mat gray;
            if (cam.channels() == 3) cv::cvtColor(cam, gray, cv::COLOR_RGB2GRAY);
            else if (cam.channels() == 4) cv::cvtColor(cam, gray, cv::COLOR_RGBA2GRAY);
            else gray = cam.clone();
            scanner.storeFrame(gray);
        }
    }
    if (state == STATE_PROJECTOR_CALIB && projectorCalib.isCapturing()) {
        if (grabber.isFrameNew() && projectorCalib.shouldCapture()) {
            cv::Mat cam = ofToCv(grabber.getPixels());
            cv::Mat gray;
            if (cam.channels() == 3) cv::cvtColor(cam, gray, cv::COLOR_RGB2GRAY);
            else if (cam.channels() == 4) cv::cvtColor(cam, gray, cv::COLOR_RGBA2GRAY);
            else gray = cam.clone();
            projectorCalib.storeCameraFrame(gray);
        }
    }

    // Update projector camera from calibration
    if (calibrationData.isComplete()) {
        // Projector extrinsics define projector pose in camera space
        // Build a glm camera that looks through the projector
        glm::vec3 t(calibrationData.T.at<double>(0, 0),
                    calibrationData.T.at<double>(1, 0),
                    calibrationData.T.at<double>(2, 0));
        cv::Mat R_t = calibrationData.R.t();
        glm::mat3 R;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                R[j][i] = (float)R_t.at<double>(i, j);

        glm::vec3 eye = t;
        glm::vec3 center = eye + R * glm::vec3(0, 0, 1);
        glm::vec3 up = R * glm::vec3(0, -1, 0);

        projectorCam.setPosition(eye);
        projectorCam.lookAt(center, up);
        projectorCam.setFov(2.0f * atan2(projH * 0.5f, calibrationData.projectorMatrix.at<double>(1, 1)) * RAD_TO_DEG);
    }
}

//--------------------------------------------------------------
void ofApp::draw() {
    ofBackground(40);

    int previewW = ofGetWidth() - 320;
    int previewH = ofGetHeight() - 100;
    int guiX = ofGetWidth() - 300;
    int guiY = 20;

    // Draw state-specific preview
    ofPushMatrix();
    ofTranslate(20, 20);
    switch (state) {
    case STATE_CAMERA_CALIB:
        cameraCalib.drawPreview(0, 0, previewW, previewH);
        cameraCalib.drawGui(guiX, guiY);
        break;
    case STATE_PROJECTOR_CALIB:
        projectorCalib.drawPreview(0, 0, previewW, previewH);
        projectorCalib.drawGui(guiX, guiY);
        break;
    case STATE_SCAN:
        scanner.drawPreview(0, 0, previewW, previewH);
        scanner.drawGui(guiX, guiY);
        break;
    case STATE_MESH_VIEW:
        meshViewer.drawGui(guiX, guiY);
        // Draw mesh preview in main window from projector POV
        ofEnableDepthTest();
        projectorCam.begin();
        meshViewer.drawProjector(projectorCam);
        projectorCam.end();
        ofDisableDepthTest();
        break;
    case STATE_MAP:
    case STATE_PERFORM:
        mappingEditor.drawGui(guiX, guiY);
        // Show projector FBO preview
        projectorFbo.draw(0, 0, previewW, previewH);
        mappingEditor.drawMaskOutlines(); // only if we track window
        break;
    }
    ofPopMatrix();

    // Draw state navigation
    drawStateGui(guiX, ofGetHeight() - 120);
}

void ofApp::drawStateGui(int x, int y) {
    stateGui.setPosition(x, y);
    stateGui.draw();
    ofDrawBitmapStringHighlight(canAdvance() ? "[Ready to advance]" : "[Complete current step]", x, y + stateGui.getHeight() + 20);
}

//--------------------------------------------------------------
void ofApp::drawProjector(ofEventArgs& args) {
    ofBackground(0);

    switch (state) {
    case STATE_PROJECTOR_CALIB:
        projectorCalib.getPatternFbo().draw(0, 0, projW, projH);
        break;
    case STATE_SCAN:
        scanner.patternFbo.draw(0, 0, projW, projH);
        break;
    case STATE_MESH_VIEW:
        if (calibrationData.isComplete()) {
            projectorFbo.begin();
            ofClear(0, 0, 0, 0);
            projectorCam.begin();
            meshViewer.drawProjector(projectorCam);
            projectorCam.end();
            projectorFbo.end();
            projectorFbo.draw(0, 0);
        }
        break;
    case STATE_MAP:
    case STATE_PERFORM:
        mappingEditor.drawProjector(projectorFbo, meshViewer.getMesh());
        projectorFbo.draw(0, 0);
        break;
    default:
        ofSetColor(255);
        ofDrawBitmapString("Projector output idle", 20, 40);
        break;
    }
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {
    switch (state) {
    case STATE_CAMERA_CALIB:
        if (key == ' ') cameraCalib.captureFrame();
        if (key == 's') cameraCalib.save(calibPath, calibrationData);
        break;
    case STATE_PROJECTOR_CALIB:
        if (key == 'c') projectorCalib.calibrate(calibrationData);
        if (key == 's') projectorCalib.save(calibPath, calibrationData);
        if (key == OF_KEY_ESC) projectorCalib.abortSequence();
        if (key == ' ' && projectorCalib.isSequencePaused()) projectorCalib.resumeSequence();
        break;
    case STATE_SCAN:
        if (key == OF_KEY_ESC) scanner.abortScan();
        if (key == ' ' && scanner.isScanPaused()) scanner.resumeScan();
        break;
    case STATE_MAP:
        mappingEditor.keyPressed(key);
        break;
    }

    if (key == OF_KEY_RIGHT || key == 'n') nextState();
    if (key == OF_KEY_LEFT || key == 'p') prevState();
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key) {
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y) {
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {
    if (state == STATE_MAP) {
        mappingEditor.mouseDragged(x, y);
    }
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {
    if (state == STATE_MAP) {
        mappingEditor.mousePressed(x, y);
    }
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {
    if (state == STATE_MAP) {
        mappingEditor.mouseReleased(x, y);
    }
}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y) {}
void ofApp::mouseExited(int x, int y) {}
void ofApp::windowResized(int w, int h) {}
void ofApp::gotMessage(ofMessage msg) {}
void ofApp::dragEvent(ofDragInfo dragInfo) {}

//--------------------------------------------------------------
bool ofApp::canAdvance() const {
    switch (state) {
    case STATE_CAMERA_CALIB: return cameraCalib.isCalibrated();
    case STATE_PROJECTOR_CALIB: return projectorCalib.isCalibrated();
    case STATE_SCAN: return scanner.isComplete();
    case STATE_MESH_VIEW: return meshViewer.hasMesh();
    case STATE_MAP: return true;
    case STATE_PERFORM: return true;
    }
    return false;
}

void ofApp::nextState() {
    if (!canAdvance()) {
        ofLogWarning("ofApp") << "Cannot advance from current state";
        return;
    }
    switch (state) {
    case STATE_CAMERA_CALIB:
        cameraCalib.save(calibPath, calibrationData);
        state = STATE_PROJECTOR_CALIB;
        break;
    case STATE_PROJECTOR_CALIB:
        projectorCalib.save(calibPath, calibrationData);
        state = STATE_SCAN;
        break;
    case STATE_SCAN:
        scanner.triangulate(calibrationData);
        meshViewer.loadMesh(scanner.getMesh());
        state = STATE_MESH_VIEW;
        break;
    case STATE_MESH_VIEW:
        state = STATE_MAP;
        break;
    case STATE_MAP:
        state = STATE_PERFORM;
        break;
    case STATE_PERFORM:
        break;
    }
    updateState();
}

void ofApp::prevState() {
    switch (state) {
    case STATE_CAMERA_CALIB: break;
    case STATE_PROJECTOR_CALIB: state = STATE_CAMERA_CALIB; break;
    case STATE_SCAN: state = STATE_PROJECTOR_CALIB; break;
    case STATE_MESH_VIEW: state = STATE_SCAN; break;
    case STATE_MAP: state = STATE_MESH_VIEW; break;
    case STATE_PERFORM: state = STATE_MAP; break;
    }
    updateState();
}

void ofApp::updateState() {
    const char* names[] = {"CAMERA_CALIB", "PROJECTOR_CALIB", "SCAN", "MESH_VIEW", "MAP", "PERFORM"};
    stateLabel = names[state];
}
