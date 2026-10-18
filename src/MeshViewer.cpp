#include "MeshViewer.h"

void MeshViewer::setup() {
    gui.setName("Mesh Viewer");
    gui.setup();
    gui.add(meshScale.setup("Scale", 1.0f, 0.1f, 5.0f));
    gui.add(wireframe.setup("Wireframe", false));
    gui.add(autoRotate.setup("Auto Rotate", false));
    meshColor.set("Color", ofColor(200, 200, 200), ofColor(0, 0, 0), ofColor(255, 255, 255));
    gui.add(meshColor);
}

void MeshViewer::update() {
    if (autoRotate) {
        orbitCam.rotateDeg(0.2f, 0, 1, 0);
    }
}

void MeshViewer::drawGui(int x, int y) {
    gui.setPosition(x, y);
    gui.draw();
    ofDrawBitmapStringHighlight("Vertices: " + ofToString(mesh.getNumVertices()), x, y + gui.getHeight() + 20);
    ofDrawBitmapStringHighlight("Triangles: " + ofToString(mesh.getNumIndices() / 3), x, y + gui.getHeight() + 40);
}

void MeshViewer::drawProjector(ofCamera& projectorCam) {
    if (!hasMesh_) return;

    ofPushStyle();
    ofSetColor(meshColor);

    ofScale(meshScale, meshScale, meshScale);

    if (wireframe) {
        mesh.drawWireframe();
    } else {
        ofEnableLighting();
        ofLight light;
        light.setPosition(0, 500, 500);
        light.enable();
        mesh.draw();
        light.disable();
        ofDisableLighting();
    }

    ofPopStyle();
}

void MeshViewer::loadMesh(ofVboMesh& m) {
    mesh = m;
    hasMesh_ = true;
}

void MeshViewer::clear() {
    mesh.clear();
    hasMesh_ = false;
}
