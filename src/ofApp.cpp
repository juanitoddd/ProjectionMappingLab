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

    // Setup projector FBO with stencil buffer for mask regions
    ofFbo::Settings fboSettings;
    fboSettings.width = projW;
    fboSettings.height = projH;
    fboSettings.internalformat = GL_RGBA8;
    fboSettings.numSamples = 0;
    fboSettings.useDepth = true;
    fboSettings.useStencil = true;
    fboSettings.depthStencilAsTexture = false;
    fboSettings.textureTarget = GL_TEXTURE_2D;
    projectorFbo.allocate(fboSettings);

    // Setup managers
    cameraCalib.setup(camW, camH);
    projectorCalib.setup(projW, projH);
    scanner.setup(projW, projH);
    meshViewer.setup();
    mappingEditor.setup(projW, projH);
    verifier.setup(9, 6, 2.5f);

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
        case STATE_VERIFY:
            verifier.update(px);
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
        // ===================================================================
        // OpenCV ↔ openFrameworks coordinate convention conversion
        // ===================================================================
        // OpenCV stereoCalibrate returns R, T that transform points FROM the
        // camera coordinate system TO the projector coordinate system:
        //     X_proj = R * X_cam + T
        //
        // OpenCV camera coords:  X right, Y DOWN, Z forward (into scene)
        // OF/GL world coords:    X right, Y UP,    Z forward (into scene)
        //
        // Conversion rule: (x, y, z)_cv → (x, -y, z)_of  (flip Y)
        //
        // Projector position in camera coords:  -R^T * T
        // Projector forward (+Z) in camera:      R^T * (0,0,1)  = third row of R
        // Projector up (-Y in cv = up in image):  R^T * (0,-1,0) = -second row of R
        //
        // After Y-flip for OF world:
        //   eye    = (-R^T * T).x,  -(-R^T * T).y,  (-R^T * T).z
        //   forward = R[2,0],       -R[2,1],        R[2,2]
        //   up      = -R[1,0],      +R[1,1],        -R[1,2]
        // ===================================================================

        cv::Mat Rcv = calibrationData.R;
        cv::Mat Rcv_T = Rcv.t();
        cv::Mat pos_cv = -Rcv_T * calibrationData.T; // 3x1

        glm::vec3 eye(
            pos_cv.at<double>(0, 0),
            -pos_cv.at<double>(1, 0),   // flip Y
            pos_cv.at<double>(2, 0)
        );

        glm::vec3 forward(
            Rcv.at<double>(2, 0),
            -Rcv.at<double>(2, 1),      // flip Y
            Rcv.at<double>(2, 2)
        );

        glm::vec3 up(
            -Rcv.at<double>(1, 0),
            +Rcv.at<double>(1, 1),      // -(-Y) = +Y
            -Rcv.at<double>(1, 2)
        );

        glm::vec3 center = eye + forward;

        projectorCam.setPosition(eye);
        projectorCam.lookAt(center, up);

        double fy = calibrationData.projectorMatrix.at<double>(1, 1);
        float fov = 2.0f * atan2(projH * 0.5f, fy) * RAD_TO_DEG;
        projectorCam.setFov(fov);

        ofLogNotice("ofApp") << "Projector pose: eye=(" << eye.x << ", " << eye.y << ", " << eye.z
            << ") fov=" << fov << "° up=(" << up.x << ", " << up.y << ", " << up.z << ")";
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
        // Show last verification result as overlay
        if (verifier.hasResult()) {
            const auto& res = verifier.getResult();
            ofColor statusColor = res.passed ? ofColor(0, 255, 0) : ofColor(255, 200, 0);
            std::string statusText = res.passed ? "PASS" : "CHECK RIG";
            ofDrawBitmapStringHighlight("Verify: RMS=" + ofToString(res.rmsError, 2) + "px [" + statusText + "]",
                20, previewH - 20, statusColor, ofColor(0));
        }
        break;
    case STATE_VERIFY:
        verifier.drawPreview(0, 0, previewW, previewH);
        verifier.drawGui(guiX, guiY);
        break;
    case STATE_MAP:
    case STATE_PERFORM:
        mappingEditor.drawGui(guiX, guiY);
        // Render projector output with camera, then draw as preview
        if (calibrationData.isComplete()) {
            projectorFbo.begin();
            projectorCam.begin();
            mappingEditor.drawProjector(projectorFbo, meshViewer.getMesh(), projectorCam.getPosition());
            projectorCam.end();
            projectorFbo.end();
        }
        projectorFbo.draw(0, 0, previewW, previewH);
        mappingEditor.setPreviewBounds(20, 20, previewW, previewH);
        mappingEditor.drawPreviewOverlay(meshViewer.getMesh());
        mappingEditor.drawMaskOutlines();
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
    case STATE_VERIFY:
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
        // Projector window just blits the pre-rendered FBO
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
    case STATE_VERIFY:
        if (key == ' ') verifier.captureVerifyPose(calibrationData);
        if (key == 'c') verifier.computeReprojectionError(calibrationData);
        if (key == 'a') verifier.applyFineTuningOffsets(calibrationData);
        if (key == 's') calibrationData.save(calibPath);
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
// Mouse coordinate correction for STATE_MAP preview area.
// The preview is at window offset (20,20) with size previewW x previewH.
//--------------------------------------------------------------
static inline void toPreviewCoords(int rawX, int rawY, int& outX, int& outY) {
    outX = rawX - 20;
    outY = rawY - 20;
    // Clamp negative values so the hit test still works at edges
    if (outX < 0) outX = 0;
    if (outY < 0) outY = 0;
}

void ofApp::mouseDragged(int x, int y, int button) {
    if (state == STATE_MAP) {
        int px, py;
        toPreviewCoords(x, y, px, py);
        mappingEditor.mouseDragged(px, py);
    }
}

void ofApp::mousePressed(int x, int y, int button) {
    if (state == STATE_MAP) {
        int px, py;
        toPreviewCoords(x, y, px, py);
        mappingEditor.mousePressed(px, py);
    }
}

void ofApp::mouseReleased(int x, int y, int button) {
    if (state == STATE_MAP) {
        int px, py;
        toPreviewCoords(x, y, px, py);
        mappingEditor.mouseReleased(px, py);
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
    case STATE_VERIFY: return true;
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
        state = STATE_VERIFY;
        break;
    case STATE_VERIFY:
        state = STATE_MAP;
        mappingEditor.computeTriangleMembership(meshViewer.getMesh());
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
    case STATE_VERIFY: state = STATE_MESH_VIEW; break;
    case STATE_MAP: state = STATE_VERIFY; break;
    case STATE_PERFORM: state = STATE_MAP; break;
    }
    updateState();
}

void ofApp::updateState() {
    const char* names[] = {"CAMERA_CALIB", "PROJECTOR_CALIB", "SCAN", "MESH_VIEW", "VERIFY", "MAP", "PERFORM"};
    stateLabel = names[state];
}
