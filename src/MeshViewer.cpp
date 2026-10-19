#include "MeshViewer.h"

void MeshViewer::setup() {
    gui.setName("Mesh Viewer");
    gui.setup();
    gui.add(meshScale.setup("Scale", 1.0f, 0.1f, 5.0f));
    gui.add(wireframe.setup("Wireframe", false));
    gui.add(autoRotate.setup("Auto Rotate", false));
    gui.add(showDiagnosticGrid.setup("Diagnostic Grid", false));
    gui.add(diagnosticGridDistance.setup("Grid Distance", 100.0f, 10.0f, 500.0f));
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

    if (showDiagnosticGrid) {
        drawDiagnosticGrid(projectorCam);
    }

    ofPopStyle();
}

void MeshViewer::drawDiagnosticGrid(ofCamera& projectorCam) {
    // Rebuild grid at current distance
    float d = diagnosticGridDistance * meshScale;
    int cells = 20;
    float size = d * 1.5f; // grid extends 1.5x distance in each direction
    float step = size / cells;

    diagnosticGrid.clear();
    diagnosticGrid.setMode(OF_PRIMITIVE_LINES);

    // Horizontal lines
    for (int i = -cells; i <= cells; ++i) {
        float y = i * step;
        diagnosticGrid.addVertex(glm::vec3(-size, y, d));
        diagnosticGrid.addVertex(glm::vec3(size, y, d));
    }
    // Vertical lines
    for (int i = -cells; i <= cells; ++i) {
        float x = i * step;
        diagnosticGrid.addVertex(glm::vec3(x, -size, d));
        diagnosticGrid.addVertex(glm::vec3(x, size, d));
    }
    // Color all vertices green
    for (size_t i = 0; i < diagnosticGrid.getNumVertices(); ++i) {
        diagnosticGrid.addColor(ofColor(0, 255, 0));
    }

    ofSetColor(0, 255, 0);
    ofSetLineWidth(2);
    diagnosticGrid.drawWireframe();
    ofSetLineWidth(1);
}

void MeshViewer::loadMesh(ofVboMesh& m) {
    mesh = m;
    hasMesh_ = true;
}

void MeshViewer::clear() {
    mesh.clear();
    hasMesh_ = false;
}
