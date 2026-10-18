#include "ofMain.h"
#include "ofApp.h"

//========================================================================
int main() {
    ofGLFWWindowSettings guiSettings;
    guiSettings.setSize(1280, 800);
    guiSettings.setGLVersion(3, 3);
    guiSettings.windowMode = OF_WINDOW;
    guiSettings.title = "ProjectionMappingLab - GUI";
    auto guiWindow = ofCreateWindow(guiSettings);

    ofGLFWWindowSettings projectorSettings;
    projectorSettings.setSize(1920, 1080);
    projectorSettings.setGLVersion(3, 3);
    projectorSettings.windowMode = OF_FULLSCREEN;
    projectorSettings.title = "ProjectionMappingLab - Projector";
    projectorSettings.monitor = 1; // try second monitor, fallback to same
    auto projectorWindow = ofCreateWindow(projectorSettings);

    auto app = std::make_shared<ofApp>();
    app->projectorWindow = projectorWindow;

    ofRunApp(guiWindow, app);
    ofRunApp(projectorWindow, app);
    ofRunMainLoop();
}
