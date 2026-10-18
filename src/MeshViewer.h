#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "StructuredLightScanner.h"

class MeshViewer {
public:
    void setup();
    void update();
    void drawGui(int x, int y);
    void drawProjector(ofCamera& projectorCam);
    void loadMesh(ofVboMesh& m);
    ofVboMesh& getMesh() { return mesh; }
    bool hasMesh() const { return hasMesh_; }
    void clear();

private:
    ofVboMesh mesh;
    bool hasMesh_ = false;

    ofEasyCam orbitCam;
    ofxPanel gui;
    ofxFloatSlider meshScale;
    ofxToggle wireframe;
    ofxToggle autoRotate;
    ofParameter<ofColor> meshColor;
};
