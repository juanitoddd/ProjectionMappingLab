#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "ofFbo.h"

struct MaskRegion {
    int id;
    std::vector<glm::vec2> vertices;           // in normalized projector space [0,1]
    std::string shaderPath;
    ofShader shader;
    bool closed = false;
};

class MappingEditor {
public:
    void setup(int projectorWidth, int projectorHeight);
    void update(float time);
    void drawGui(int x, int y);
    void drawProjector(ofFbo& projectorFbo, ofVboMesh& sceneMesh);
    void mousePressed(int x, int y);
    void mouseDragged(int x, int y);
    void mouseReleased(int x, int y);
    void keyPressed(int key);

    std::vector<MaskRegion>& getRegions() { return regions; }
    void loadShaders();
    void drawMaskOutlines();

private:
    int projW, projH;
    std::vector<MaskRegion> regions;
    bool drawing = false;
    int selectedRegion = -1;
    bool showMasks = true;

    ofFbo maskFbo;
    ofxPanel gui;
    ofxButton btnNewMask, btnCloseMask, btnDeleteMask, btnReloadShaders;
    ofxToggle tglShowMasks;

    void onNewMask();
    void onCloseMask();
    void onDeleteMask();
    void beginNewMask(const glm::vec2& p);
    void closeCurrentMask();
    void deleteSelectedMask();
    void rebuildMaskFbo();
};
