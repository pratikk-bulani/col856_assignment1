#include "camera.hpp"
#include "geometry.hpp"
#include "gui.hpp"
#include "lighting.hpp"
#include "text.hpp"
#include "integrators.hpp"

#include <cmath>

using namespace std;

Window window;
Camera camera;
Lighting lighting;
Text text;

#ifdef SYMPLECTIC_EULER
float dt = 1/600.;
#endif
#ifdef BACKWARD_EULER_NEWTON
float dt = 1/60.;
#endif
float t = 0;
bool paused = false;
// vec3 p0, p1;
PointGeometry pg(11, 11);

void drawStuff() {
    setColor(vec3(0.8,0.2,0.2));
    drawArrow(vec3(0,0,0), vec3(1,0,0), 0.02);
    setColor(vec3(0.2,0.6,0.2));
    drawArrow(vec3(0,0,0), vec3(0,1,0), 0.02);
    setColor(vec3(0.2,0.2,0.8));
    drawArrow(vec3(0,0,0), vec3(0,0,1), 0.02);
    setPointSize(10);
    setColor(vec3(0.2,0.2,0.2));
    // drawPoint(p0);
    // drawPoint(p1);
    pg.draw();
}

void drawWorld() {
    camera.apply(window);
    lighting.apply();
    clear(vec3(0.9,0.9,0.9));
    setColor(vec3(0.7,0.7,0.7));
    drawQuad(vec3(-3,0,-3), vec3(-3,0,3), vec3(3,0,3), vec3(3,0,-3));
    drawStuff();
    setColor(vec3(0,0,0));
    text.draw("WASD and LShift/LCtrl to move camera", -0.9, 0.90);
    text.draw("Mouse to rotate view", -0.9, 0.85);
    text.draw("Space to play/pause animation", -0.9, 0.80);
}

void update(float dt) {
    t += dt;
    // p0 = vec3(sin(M_PI*t), 1, cos(M_PI*t));
    // p1 = vec3(-sin(M_PI*t), 1, -cos(M_PI*t));
    calculateForce(pg.mass, pg.point_forces, pg.point_coordinates, pg.springs);
    calcNextState(dt, pg);
}

void keyPressed(int key) {
    // See http://www.glfw.org/docs/latest/group__keys.html for key codes
    if (key == GLFW_KEY_SPACE)
        paused = !paused;
}

int main(int argc, char **argv) {
    window.create("Animation", 1024, 768);
    window.onKeyPress(keyPressed);
    camera.lookAt(vec3(1,1.5,5), vec3(0,0.5,0));
    lighting.createDefault();
    text.initialize();
    while (!window.shouldClose()) {
        camera.processInput(window);
        if (!paused)
            update(dt);
        window.prepareDisplay();
        drawWorld();
        window.updateDisplay(); 
        window.waitForNextFrame(dt);
    }
}
