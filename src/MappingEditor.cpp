#include "MappingEditor.h"

void MappingEditor::setup(int projectorWidth, int projectorHeight) {
    projW = projectorWidth;
    projH = projectorHeight;
    maskFbo.allocate(projW, projH, GL_RGBA);

    gui.setName("Mapping Editor");
    gui.setup();
    gui.add(btnNewMask.setup("New Mask (N)"));
    gui.add(btnCloseMask.setup("Close Mask (Enter)"));
    gui.add(btnDeleteMask.setup("Delete Mask (Del)"));
    gui.add(btnReloadShaders.setup("Reload Shaders (R)"));
    gui.add(tglShowMasks.setup("Show Masks", true));

    btnNewMask.addListener(this, &MappingEditor::onNewMask);
    btnCloseMask.addListener(this, &MappingEditor::onCloseMask);
    btnDeleteMask.addListener(this, &MappingEditor::onDeleteMask);
    btnReloadShaders.addListener(this, &MappingEditor::loadShaders);
}

void MappingEditor::update(float time) {
    // Update shader uniforms in shader cache
    for (auto& reg : regions) {
        if (reg.shader.isLoaded()) {
            // uniforms set per draw call, not cached here
        }
    }
}

void MappingEditor::drawGui(int x, int y) {
    gui.setPosition(x, y);
    gui.draw();
    ofDrawBitmapStringHighlight("Regions: " + ofToString((int)regions.size()), x, y + gui.getHeight() + 20);
    if (selectedRegion >= 0 && selectedRegion < (int)regions.size()) {
        ofDrawBitmapStringHighlight("Selected: " + ofToString(regions[selectedRegion].id), x, y + gui.getHeight() + 40);
    }
}

void MappingEditor::drawProjector(ofFbo& projectorFbo, ofVboMesh& sceneMesh) {
    projectorFbo.begin();
    ofClear(0, 0, 0, 255);
    ofEnableDepthTest();

    if (showMasks) {
        // First pass: render full scene as base
        // Then render each mask region with its shader, masked by polygon
        for (auto& reg : regions) {
            if (!reg.closed || !reg.shader.isLoaded()) continue;

            // Set stencil / mask based on polygon
            // For now, simplified: render to maskFbo, then use as alpha mask
            maskFbo.begin();
            ofClear(0, 0, 0, 0);
            ofSetColor(255);
            ofBeginShape();
            for (auto& v : reg.vertices) {
                ofVertex(v.x * projW, v.y * projH);
            }
            ofEndShape(true);
            maskFbo.end();

            reg.shader.begin();
            reg.shader.setUniform1f("u_time", ofGetElapsedTimef());
            reg.shader.setUniform2f("u_res", projW, projH);
            reg.shader.setUniform1i("u_maskID", reg.id);
            // sceneMesh.draw(); // would need projector camera setup outside
            reg.shader.end();
        }
    }

    ofDisableDepthTest();
    projectorFbo.end();
}

void MappingEditor::mousePressed(int x, int y) {
    if (!drawing) return;
    glm::vec2 p(x / (float)projW, y / (float)projH);
    if (regions.empty() || regions.back().closed) {
        beginNewMask(p);
    } else {
        regions.back().vertices.push_back(p);
    }
}

void MappingEditor::mouseDragged(int x, int y) {
    // could implement vertex drag on selected region
}

void MappingEditor::mouseReleased(int x, int y) {
}

void MappingEditor::keyPressed(int key) {
    if (key == 'n' || key == 'N') beginNewMask(glm::vec2(0.5f, 0.5f));
    if (key == OF_KEY_RETURN || key == OF_KEY_RETURN) closeCurrentMask();
    if (key == OF_KEY_DEL || key == OF_KEY_BACKSPACE) deleteSelectedMask();
    if (key == 'r' || key == 'R') loadShaders();
}

void MappingEditor::onNewMask() {
    beginNewMask(glm::vec2(0.5f, 0.5f));
}

void MappingEditor::onCloseMask() {
    closeCurrentMask();
}

void MappingEditor::onDeleteMask() {
    deleteSelectedMask();
}

void MappingEditor::beginNewMask(const glm::vec2& p) {
    MaskRegion reg;
    reg.id = (int)regions.size();
    reg.vertices.push_back(p);
    reg.closed = false;
    regions.push_back(reg);
    selectedRegion = (int)regions.size() - 1;
    drawing = true;
    ofLogNotice("MappingEditor") << "Started new mask " << reg.id;
}

void MappingEditor::closeCurrentMask() {
    if (regions.empty() || regions.back().closed) return;
    if (regions.back().vertices.size() < 3) {
        regions.pop_back();
        return;
    }
    regions.back().closed = true;
    drawing = false;
    ofLogNotice("MappingEditor") << "Closed mask " << regions.back().id;

    // Assign default shader
    regions.back().shaderPath = "shaders/default.frag";
    if (!regions.back().shader.load("shaders/default")) {
        ofLogWarning("MappingEditor") << "Could not load default shader";
    }
}

void MappingEditor::deleteSelectedMask() {
    if (selectedRegion < 0 || selectedRegion >= (int)regions.size()) return;
    regions.erase(regions.begin() + selectedRegion);
    selectedRegion = -1;
    ofLogNotice("MappingEditor") << "Deleted mask";
}

void MappingEditor::loadShaders() {
    for (auto& reg : regions) {
        if (!reg.shaderPath.empty()) {
            if (!reg.shader.load(reg.shaderPath)) {
                ofLogWarning("MappingEditor") << "Failed to load " << reg.shaderPath;
            }
        }
    }
    ofLogNotice("MappingEditor") << "Shaders reloaded";
}

void MappingEditor::drawMaskOutlines() {
    for (auto& reg : regions) {
        if (reg.vertices.empty()) continue;
        ofSetColor(reg.id == selectedRegion ? ofColor(0, 255, 0) : ofColor(255, 0, 0));
        ofSetLineWidth(2);
        ofNoFill();
        ofBeginShape();
        for (auto& v : reg.vertices) {
            ofVertex(v.x * ofGetWidth(), v.y * ofGetHeight());
        }
        if (reg.closed) {
            ofEndShape(true);
        } else {
            ofEndShape(false);
            // draw vertices
            ofFill();
            for (auto& v : reg.vertices) {
                ofDrawCircle(v.x * ofGetWidth(), v.y * ofGetHeight(), 4);
            }
        }
    }
}

void MappingEditor::rebuildMaskFbo() {
    maskFbo.begin();
    ofClear(0, 0, 0, 0);
    // rebuild mask texture per region
    maskFbo.end();
}
