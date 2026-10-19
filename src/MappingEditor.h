#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "ofFbo.h"

struct MaskRegion {
    int id;
    std::vector<glm::vec2> vertices;           // in normalized projector space [0,1]
    std::vector<int> triangleStartIndices;     // starting positions in mesh index buffer (multiples of 3)
    ofVboMesh regionMesh;                      // sub-mesh containing only this region's triangles
    std::string shaderPath;
    ofShader shader;
    bool closed = false;
};

class MappingEditor {
public:
    void setup(int projectorWidth, int projectorHeight);
    void update(float time);
    void drawGui(int x, int y);
    void drawProjector(ofFbo& projectorFbo, ofVboMesh& sceneMesh, const glm::vec3& projectorPos);
    void drawPreviewOverlay(ofVboMesh& sceneMesh); // live triangle highlight for open mask
    void setProjectorPosition(const glm::vec3& pos) { projectorPos_ = pos; }
    void setPreviewBounds(int offsetX, int offsetY, int previewW_, int previewH_);
    void mousePressed(int x, int y);
    void mouseDragged(int x, int y);
    void mouseReleased(int x, int y);
    void keyPressed(int key);

    std::vector<MaskRegion>& getRegions() { return regions; }
    void loadShaders();
    void drawMaskOutlines();
    void computeTriangleMembership(ofVboMesh& sceneMesh);

private:
    int projW, projH;
    std::vector<MaskRegion> regions;
    std::vector<int> unmaskedIndices;          // mesh triangles not in any region
    bool drawing = false;
    int selectedRegion = -1;
    bool showMasks = true;

    ofFbo maskFbo;
    ofVboMesh unmaskedMesh_;                   // cached unmasked sub-mesh (rebuilt on membership change)
    ofShader defaultShader;                    // pass-through for unmasked areas
    ofShader baseShader;                       // neutral grey for base mesh
    ofxPanel gui;
    ofxButton btnNewMask, btnCloseMask, btnDeleteMask, btnReloadShaders;
    ofxToggle tglShowMasks;

    std::string lastShaderError_;              // last compilation error, shown in GUI
    glm::vec3 projectorPos_;                   // passed as u_projectorPos

    int previewOffsetX = 0, previewOffsetY = 0;
    int previewW = 1920, previewH = 1080;

    int draggingVertexIdx = -1;                // vertex being dragged (-1 = none)
    int draggingRegionIdx = -1;                // region whose vertex is dragged
    float vertexHitRadius = 8.0f;            // px radius for vertex selection

    ofVboMesh highlightMesh_;                // temporary mesh for live triangle highlight
    bool highlightDirty = true;              // true when polygon changed, rebuild highlight

    void onNewMask();
    void onCloseMask();
    void onReopenMask();
    void onDeleteMask();
    void beginNewMask(const glm::vec2& p);
    void closeCurrentMask();
    void reopenMask(int regionIdx);
    void deleteSelectedMask();
    void rebuildMaskFbo();
    glm::vec2 screenToNorm(int sx, int sy) const; // screen → [0,1] inside preview bounds
    int findNearestVertex(int sx, int sy, int& outRegionIdx) const;
    bool pointInPolygon(const glm::vec2& p, const std::vector<glm::vec2>& poly) const;
    void buildHighlightMesh(ofVboMesh& sceneMesh);
    void buildSubMesh(ofVboMesh& subMesh, const ofVboMesh& source,
                      const std::vector<bool>& triangleClaimed, bool wantClaimed);
    void loadDefaultShaders();
    void tryLoadRegionShader(MaskRegion& reg);
    void bindBaseUniforms(ofShader& sh, int maskID);
};
