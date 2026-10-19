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

    loadDefaultShaders();
}

void MappingEditor::loadDefaultShaders() {
    if (!baseShader.load("shaders/default")) {
        ofLogWarning("MappingEditor") << "Could not load base shader (shaders/default)";
    }
    if (!defaultShader.load("shaders/default")) {
        ofLogWarning("MappingEditor") << "Could not load default region shader";
    }
}

void MappingEditor::update(float time) {
    // Nothing per-frame; uniforms bound at draw time
}

void MappingEditor::drawGui(int x, int y) {
    gui.setPosition(x, y);
    gui.draw();
    ofDrawBitmapStringHighlight("Regions: " + ofToString((int)regions.size()), x, y + gui.getHeight() + 20);
    if (selectedRegion >= 0 && selectedRegion < (int)regions.size()) {
        auto& reg = regions[selectedRegion];
        std::string status = reg.closed ? "closed" : "open";
        ofDrawBitmapStringHighlight("Selected: " + ofToString(reg.id) + " (" + status + ")  Shader: " + reg.shaderPath,
            x, y + gui.getHeight() + 40);
    } else {
        ofDrawBitmapStringHighlight("No mask selected", x, y + gui.getHeight() + 40);
    }
    if (!lastShaderError_.empty()) {
        ofDrawBitmapStringHighlight("Shader error: " + lastShaderError_, x, y + gui.getHeight() + 60, ofColor(255, 0, 0), ofColor(255));
    } else {
        ofDrawBitmapStringHighlight("Shaders: OK", x, y + gui.getHeight() + 60);
    }
    ofDrawBitmapStringHighlight("Hint: click=add vertex  drag=move vertex  Enter=close  O=reopen  Del=delete",
        x, y + gui.getHeight() + 80, ofColor(100, 100, 100), ofColor(255));
}

// ---------------------------------------------------------------------------
// Stencil-based projector rendering
// ---------------------------------------------------------------------------

void MappingEditor::drawProjector(ofFbo& projectorFbo, ofVboMesh& sceneMesh, const glm::vec3& projectorPos) {
    projectorPos_ = projectorPos;

    ofClear(0, 0, 0, 255);
    ofEnableDepthTest();

    if (!showMasks || regions.empty()) {
        baseShader.begin();
        bindBaseUniforms(baseShader, -1);
        sceneMesh.draw();
        baseShader.end();
    } else {
        // --- Pass 1: cached unmasked areas ---
        if (unmaskedMesh_.getNumVertices() > 0) {
            baseShader.begin();
            bindBaseUniforms(baseShader, -1);
            unmaskedMesh_.draw();
            baseShader.end();
        }

        // --- Pass 2: each masked region via stencil ---
        for (auto& reg : regions) {
            if (!reg.closed || reg.regionMesh.getNumVertices() == 0) continue;

            glEnable(GL_STENCIL_TEST);
            glClearStencil(0);
            glClear(GL_STENCIL_BUFFER_BIT);
            glStencilFunc(GL_ALWAYS, 1, 0xFF);
            glStencilOp(GL_REPLACE, GL_REPLACE, GL_REPLACE);
            glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
            glDepthMask(GL_FALSE);

            ofPushMatrix();
            ofSetColor(255);
            ofFill();
            ofBeginShape();
            for (auto& v : reg.vertices) {
                ofVertex(v.x * projW, v.y * projH);
            }
            ofEndShape(true);
            ofPopMatrix();

            glStencilFunc(GL_EQUAL, 1, 0xFF);
            glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
            glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
            glDepthMask(GL_TRUE);

            ofShader* sh = reg.shader.isLoaded() ? &reg.shader : &defaultShader;
            sh->begin();
            bindBaseUniforms(*sh, reg.id);
            sh->setUniform3f("u_projectorPos", projectorPos_.x, projectorPos_.y, projectorPos_.z);
            reg.regionMesh.draw();
            sh->end();

            glDisable(GL_STENCIL_TEST);
        }
    }

    ofDisableDepthTest();
}

// ---------------------------------------------------------------------------
// Live preview overlay: highlight triangles inside the open (not-yet-closed) mask
// ---------------------------------------------------------------------------

void MappingEditor::drawPreviewOverlay(ofVboMesh& sceneMesh) {
    // Only highlight if there's an open mask being drawn
    int openIdx = -1;
    for (int i = 0; i < (int)regions.size(); ++i) {
        if (!regions[i].closed && regions[i].vertices.size() >= 3) {
            openIdx = i;
            break;
        }
    }
    if (openIdx < 0) return;

    if (highlightDirty) {
        buildHighlightMesh(sceneMesh);
        highlightDirty = false;
    }

    if (highlightMesh_.getNumVertices() > 0) {
        ofPushStyle();
        ofSetColor(0, 255, 255, 120); // cyan tint
        ofSetLineWidth(1);
        ofNoFill();
        highlightMesh_.drawWireframe();
        ofPopStyle();
    }
}

void MappingEditor::buildHighlightMesh(ofVboMesh& sceneMesh) {
    highlightMesh_.clear();
    highlightMesh_.setMode(OF_PRIMITIVE_TRIANGLES);

    if (sceneMesh.getNumIndices() == 0) return;

    int triCount = sceneMesh.getNumIndices() / 3;
    const auto& indices = sceneMesh.getIndices();
    const auto& texcoords = sceneMesh.getTexCoords();

    // Find the open mask
    const MaskRegion* openMask = nullptr;
    for (auto& reg : regions) {
        if (!reg.closed && reg.vertices.size() >= 3) {
            openMask = &reg;
            break;
        }
    }
    if (!openMask) return;

    for (int tri = 0; tri < triCount; ++tri) {
        int i0 = indices[tri * 3];
        int i1 = indices[tri * 3 + 1];
        int i2 = indices[tri * 3 + 2];

        glm::vec2 centroid(0.0f);
        if (i0 < (int)texcoords.size()) centroid += texcoords[i0];
        if (i1 < (int)texcoords.size()) centroid += texcoords[i1];
        if (i2 < (int)texcoords.size()) centroid += texcoords[i2];
        centroid /= 3.0f;

        if (pointInPolygon(centroid, openMask->vertices)) {
            for (int k = 0; k < 3; ++k) {
                int idx = indices[tri * 3 + k];
                highlightMesh_.addVertex(sceneMesh.getVertex(idx));
                if (idx < (int)sceneMesh.getNormals().size())
                    highlightMesh_.addNormal(sceneMesh.getNormal(idx));
            }
        }
    }
}

void MappingEditor::bindBaseUniforms(ofShader& sh, int maskID) {
    sh.setUniform1f("u_time", ofGetElapsedTimef());
    sh.setUniform2f("u_res", projW, projH);
    sh.setUniform2f("u_meshUV", 1.0f, 1.0f);
    sh.setUniform1i("u_maskID", maskID);
}

// ---------------------------------------------------------------------------
// Sub-mesh helpers
// ---------------------------------------------------------------------------

void MappingEditor::buildSubMesh(ofVboMesh& subMesh, const ofVboMesh& source,
                                 const std::vector<bool>& triangleClaimed, bool wantClaimed) {
    subMesh.clear();
    subMesh.setMode(OF_PRIMITIVE_TRIANGLES);

    int triCount = source.getNumIndices() / 3;
    const auto& srcVerts = source.getVertices();
    const auto& srcTex = source.getTexCoords();
    const auto& srcNorm = source.getNormals();
    const auto& srcColors = source.getColors();
    const auto& srcIndices = source.getIndices();

    for (int tri = 0; tri < triCount; ++tri) {
        bool claimed = (tri < (int)triangleClaimed.size()) ? triangleClaimed[tri] : false;
        if (claimed != wantClaimed) continue;

        for (int k = 0; k < 3; ++k) {
            int idx = srcIndices[tri * 3 + k];
            subMesh.addVertex(srcVerts[idx]);
            if (idx < (int)srcTex.size()) subMesh.addTexCoord(srcTex[idx]);
            if (idx < (int)srcNorm.size()) subMesh.addNormal(srcNorm[idx]);
            if (idx < (int)srcColors.size()) subMesh.addColor(srcColors[idx]);
        }
    }
}

// ---------------------------------------------------------------------------
// Triangle membership
// ---------------------------------------------------------------------------

void MappingEditor::computeTriangleMembership(ofVboMesh& sceneMesh) {
    if (sceneMesh.getNumIndices() == 0) return;

    int triCount = sceneMesh.getNumIndices() / 3;
    const auto& indices = sceneMesh.getIndices();
    const auto& texcoords = sceneMesh.getTexCoords();

    for (auto& reg : regions) {
        reg.triangleStartIndices.clear();
        reg.regionMesh.clear();
    }

    std::vector<bool> triangleClaimed;
    triangleClaimed.resize(triCount, false);

    for (int tri = 0; tri < triCount; ++tri) {
        int i0 = indices[tri * 3];
        int i1 = indices[tri * 3 + 1];
        int i2 = indices[tri * 3 + 2];

        glm::vec2 centroid(0.0f);
        if (i0 < (int)texcoords.size()) centroid += texcoords[i0];
        if (i1 < (int)texcoords.size()) centroid += texcoords[i1];
        if (i2 < (int)texcoords.size()) centroid += texcoords[i2];
        centroid /= 3.0f;

        for (auto& reg : regions) {
            if (!reg.closed) continue;
            if (pointInPolygon(centroid, reg.vertices)) {
                reg.triangleStartIndices.push_back(tri * 3);
                triangleClaimed[tri] = true;
                break;
            }
        }
    }

    // Build sub-meshes
    for (auto& reg : regions) {
        if (!reg.closed || reg.triangleStartIndices.empty()) continue;
        reg.regionMesh.clear();
        reg.regionMesh.setMode(OF_PRIMITIVE_TRIANGLES);
        for (int start : reg.triangleStartIndices) {
            for (int k = 0; k < 3; ++k) {
                int idx = indices[start + k];
                reg.regionMesh.addVertex(sceneMesh.getVertex(idx));
                if (idx < (int)sceneMesh.getTexCoords().size())
                    reg.regionMesh.addTexCoord(sceneMesh.getTexCoord(idx));
                if (idx < (int)sceneMesh.getNormals().size())
                    reg.regionMesh.addNormal(sceneMesh.getNormal(idx));
                if (idx < (int)sceneMesh.getColors().size())
                    reg.regionMesh.addColor(sceneMesh.getColor(idx));
            }
        }
    }

    // Cache unmasked mesh once
    unmaskedMesh_.clear();
    if (sceneMesh.getNumIndices() > 0) {
        buildSubMesh(unmaskedMesh_, sceneMesh, triangleClaimed, true);
    }

    ofLogNotice("MappingEditor") << "Membership computed: " << triCount << " triangles, "
        << (unmaskedMesh_.getNumVertices() / 3) << " unmasked";
}

bool MappingEditor::pointInPolygon(const glm::vec2& p, const std::vector<glm::vec2>& poly) const {
    if (poly.size() < 3) return false;
    bool inside = false;
    for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
        const glm::vec2& a = poly[i];
        const glm::vec2& b = poly[j];
        if (((a.y > p.y) != (b.y > p.y)) &&
            (p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x)) {
            inside = !inside;
        }
    }
    return inside;
}

// ---------------------------------------------------------------------------
// Coordinate mapping
// ---------------------------------------------------------------------------

void MappingEditor::setPreviewBounds(int offsetX, int offsetY, int previewW_, int previewH_) {
    previewOffsetX = offsetX;
    previewOffsetY = offsetY;
    previewW = previewW_;
    previewH = previewH_;
}

glm::vec2 MappingEditor::screenToNorm(int sx, int sy) const {
    // sx, sy are already offset-corrected by caller (window → preview local)
    // Map from preview pixel space to projector-normalized [0,1]
    float nx = sx / (float)previewW;
    float ny = sy / (float)previewH;
    // Clamp
    if (nx < 0.0f) nx = 0.0f; if (nx > 1.0f) nx = 1.0f;
    if (ny < 0.0f) ny = 0.0f; if (ny > 1.0f) ny = 1.0f;
    return glm::vec2(nx, ny);
}

// ---------------------------------------------------------------------------
// Vertex hit test: return closest vertex within vertexHitRadius pixels
// ---------------------------------------------------------------------------

int MappingEditor::findNearestVertex(int sx, int sy, int& outRegionIdx) const {
    float bestDist = vertexHitRadius * vertexHitRadius;
    int bestIdx = -1;
    outRegionIdx = -1;

    for (int r = 0; r < (int)regions.size(); ++r) {
        const auto& reg = regions[r];
        for (int v = 0; v < (int)reg.vertices.size(); ++v) {
            float vx = reg.vertices[v].x * previewW;
            float vy = reg.vertices[v].y * previewH;
            float dx = vx - sx;
            float dy = vy - sy;
            float dist2 = dx * dx + dy * dy;
            if (dist2 < bestDist) {
                bestDist = dist2;
                bestIdx = v;
                outRegionIdx = r;
            }
        }
    }
    return bestIdx;
}

// ---------------------------------------------------------------------------
// Input handling
// ---------------------------------------------------------------------------

void MappingEditor::mousePressed(int sx, int sy) {
    // First: try to select a vertex for dragging
    int regionIdx = -1;
    int vertexIdx = findNearestVertex(sx, sy, regionIdx);

    if (vertexIdx >= 0) {
        // Start dragging an existing vertex
        draggingVertexIdx = vertexIdx;
        draggingRegionIdx = regionIdx;
        selectedRegion = regionIdx;
        return;
    }

    // If a closed mask was clicked (not on a vertex), select it
    glm::vec2 p = screenToNorm(sx, sy);
    for (int i = (int)regions.size() - 1; i >= 0; --i) {
        if (regions[i].closed && pointInPolygon(p, regions[i].vertices)) {
            selectedRegion = i;
            return;
        }
    }

    // Otherwise: add a vertex to the current open mask, or start a new one
    if (regions.empty() || regions.back().closed) {
        beginNewMask(p);
    } else {
        regions.back().vertices.push_back(p);
        highlightDirty = true;
    }
}

void MappingEditor::mouseDragged(int sx, int sy) {
    if (draggingVertexIdx >= 0 && draggingRegionIdx >= 0 && draggingRegionIdx < (int)regions.size()) {
        auto& reg = regions[draggingRegionIdx];
        if (draggingVertexIdx < (int)reg.vertices.size()) {
            reg.vertices[draggingVertexIdx] = screenToNorm(sx, sy);
            highlightDirty = true;
        }
    }
}

void MappingEditor::mouseReleased(int sx, int sy) {
    draggingVertexIdx = -1;
    draggingRegionIdx = -1;
}

void MappingEditor::keyPressed(int key) {
    if (key == 'n' || key == 'N') beginNewMask(glm::vec2(0.5f, 0.5f));
    if (key == OF_KEY_RETURN || key == OF_KEY_RETURN) closeCurrentMask();
    if (key == 'o' || key == 'O') {
        if (selectedRegion >= 0) reopenMask(selectedRegion);
    }
    if (key == OF_KEY_DEL || key == OF_KEY_BACKSPACE) deleteSelectedMask();
    if (key == 'r' || key == 'R') loadShaders();
}

void MappingEditor::onNewMask() { beginNewMask(glm::vec2(0.5f, 0.5f)); }
void MappingEditor::onCloseMask() { closeCurrentMask(); }
void MappingEditor::onReopenMask() {
    if (selectedRegion >= 0) reopenMask(selectedRegion);
}
void MappingEditor::onDeleteMask() { deleteSelectedMask(); }

void MappingEditor::beginNewMask(const glm::vec2& p) {
    MaskRegion reg;
    reg.id = (int)regions.size();
    reg.vertices.push_back(p);
    reg.closed = false;
    regions.push_back(reg);
    selectedRegion = (int)regions.size() - 1;
    drawing = true;
    highlightDirty = true;
    ofLogNotice("MappingEditor") << "Started new mask " << reg.id;
}

void MappingEditor::closeCurrentMask() {
    // Close the currently open mask (last one if not closed)
    int openIdx = -1;
    for (int i = 0; i < (int)regions.size(); ++i) {
        if (!regions[i].closed) {
            openIdx = i;
            break;
        }
    }
    if (openIdx < 0) return;

    auto& reg = regions[openIdx];
    if (reg.vertices.size() < 3) {
        regions.erase(regions.begin() + openIdx);
        if (selectedRegion == openIdx) selectedRegion = -1;
        return;
    }
    reg.closed = true;
    drawing = false;
    highlightDirty = true;
    ofLogNotice("MappingEditor") << "Closed mask " << reg.id;

    tryLoadRegionShader(reg);
}

void MappingEditor::reopenMask(int regionIdx) {
    if (regionIdx < 0 || regionIdx >= (int)regions.size()) return;
    if (!regions[regionIdx].closed) return;

    regions[regionIdx].closed = false;
    regions[regionIdx].regionMesh.clear();
    regions[regionIdx].triangleStartIndices.clear();
    selectedRegion = regionIdx;
    drawing = true;
    highlightDirty = true;
    ofLogNotice("MappingEditor") << "Reopened mask " << regions[regionIdx].id << " for editing";
}

void MappingEditor::tryLoadRegionShader(MaskRegion& reg) {
    std::string path = "shaders/effect" + (reg.id == 0 ? "" : ofToString(reg.id));

    ofShader test;
    if (test.load(path)) {
        reg.shader = test;
        reg.shaderPath = path;
        ofLogNotice("MappingEditor") << "Loaded region " << reg.id << " shader: " << path;
    } else {
        if (reg.id != 0) {
            ofShader fallback;
            if (fallback.load("shaders/effect")) {
                reg.shader = fallback;
                reg.shaderPath = "shaders/effect";
                ofLogNotice("MappingEditor") << "Region " << reg.id << " using generic effect shader";
                return;
            }
        }
        reg.shader = defaultShader;
        reg.shaderPath = "shaders/default";
        ofLogWarning("MappingEditor") << "Could not load shader for region " << reg.id << ", using default";
    }
}

void MappingEditor::deleteSelectedMask() {
    if (selectedRegion < 0 || selectedRegion >= (int)regions.size()) return;
    regions.erase(regions.begin() + selectedRegion);
    selectedRegion = -1;
    highlightDirty = true;
    ofLogNotice("MappingEditor") << "Deleted mask";
}

void MappingEditor::loadShaders() {
    loadDefaultShaders();
    lastShaderError_.clear();

    for (auto& reg : regions) {
        if (!reg.shaderPath.empty()) {
            ofShader test;
            if (test.load(reg.shaderPath)) {
                reg.shader = test;
            } else {
                lastShaderError_ = reg.shaderPath + " compile/link failed";
                ofLogWarning("MappingEditor") << lastShaderError_;
                reg.shader = defaultShader;
            }
        }
    }
    ofLogNotice("MappingEditor") << "Shaders reloaded";
}

// ---------------------------------------------------------------------------
// Mask outlines — drawn in screen space aligned with the preview area
// ---------------------------------------------------------------------------

void MappingEditor::drawMaskOutlines() {
    // Preview FBO was drawn at (0,0) inside a ofTranslate(20,20) block,
    // with size previewW x previewH.  So outlines must map vertices
    // (normalized [0,1]) → preview pixel coords.
    for (int r = 0; r < (int)regions.size(); ++r) {
        auto& reg = regions[r];
        if (reg.vertices.empty()) continue;

        bool isSelected = (r == selectedRegion);
        bool isOpen = !reg.closed;

        if (isSelected && isOpen) {
            ofSetColor(0, 255, 255); // cyan for active open mask
        } else if (isSelected) {
            ofSetColor(0, 255, 0); // green for selected closed mask
        } else if (isOpen) {
            ofSetColor(255, 165, 0); // orange for other open mask
        } else {
            ofSetColor(255, 0, 0); // red for unselected closed masks
        }

        ofSetLineWidth(isSelected ? 3 : 1);
        ofNoFill();
        ofBeginShape();
        for (auto& v : reg.vertices) {
            ofVertex(v.x * previewW, v.y * previewH);
        }
        if (reg.closed) {
            ofEndShape(true);
        } else {
            ofEndShape(false);
            // Draw vertex handles
            ofFill();
            for (size_t i = 0; i < reg.vertices.size(); ++i) {
                float vx = reg.vertices[i].x * previewW;
                float vy = reg.vertices[i].y * previewH;
                if (isSelected && (int)i == draggingVertexIdx) {
                    ofSetColor(255, 255, 0);
                    ofDrawCircle(vx, vy, 6);
                } else {
                    ofSetColor(isSelected ? ofColor(200, 200, 255) : ofColor(200));
                    ofDrawCircle(vx, vy, 4);
                }
            }
        }
    }
}

void MappingEditor::rebuildMaskFbo() {
    maskFbo.begin();
    ofClear(0, 0, 0, 0);
    maskFbo.end();
}
