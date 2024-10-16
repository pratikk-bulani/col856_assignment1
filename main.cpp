#include "camera.hpp"
#include "draw.hpp"
#include "gui.hpp"
#include "lighting.hpp"
#include "text.hpp"
#include "forces.hpp"
#include "integrators.hpp"

#include <cmath>

using namespace std;

Window window;
Camera camera;
Lighting lighting;
Text text;

float dt = 1/600.;
float t = 0;
bool paused = false;
// vec3 p0, p1;
int x_max = 11, y_max = 11;
vector<vector<vec3>> cloth_points_coordinates(y_max, vector<vec3>(x_max, vec3(0., 0., 0.)));
vector<vector<vec3>> cloth_points_velocities(y_max, vector<vec3>(x_max, vec3(0., 0., 0.)));
vector<vector<vec3>> cloth_points_forces(y_max, vector<vec3>(x_max, vec3(0., 0., 0.)));
double mass = 0.0;

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
    for(int i = 0; i < y_max; i++) {
        for(int j = 0; j < x_max; j++) {
            drawPoint(cloth_points_coordinates[i][j]);
        }
    }
    for(int i = 0; i < y_max - 1; i++) {
        for(int j = 0; j < x_max - 1; j++) {
            drawLine(cloth_points_coordinates[i][j], cloth_points_coordinates[i+1][j]);
            drawLine(cloth_points_coordinates[i][j], cloth_points_coordinates[i][j+1]);
            drawLine(cloth_points_coordinates[i][j], cloth_points_coordinates[i+1][j+1]);
        }
    }
    for(int i = y_max - 1; i > 0; i--) {
        for(int j = 0; j < x_max - 1; j++) {
            drawLine(cloth_points_coordinates[i][j], cloth_points_coordinates[i-1][j+1]);
        }
    }
    for(int i = 0; i < x_max - 1; i++) {
        drawLine(cloth_points_coordinates[y_max - 1][i], cloth_points_coordinates[y_max - 1][i+1]);
    }
    for(int i = 0; i < y_max - 1; i++) {
        drawLine(cloth_points_coordinates[i][x_max - 1], cloth_points_coordinates[i+1][x_max - 1]);
    }

    // // Two hops away
    // for(int i = 0; i < y_max - 2; i++) {
    //     for(int j = 0; j < x_max - 2; j++) {
    //         drawLine(cloth_points_coordinates[i][j], cloth_points_coordinates[i+2][j]);
    //         drawLine(cloth_points_coordinates[i][j], cloth_points_coordinates[i][j+2]);
    //     }
    // }
    // for(int i = 0; i < x_max - 2; i++) {
    //     drawLine(cloth_points_coordinates[y_max - 1][i], cloth_points_coordinates[y_max - 1][i+2]);
    //     drawLine(cloth_points_coordinates[y_max - 2][i], cloth_points_coordinates[y_max - 2][i+2]);
    // }
    // for(int i = 0; i < y_max - 2; i++) {
    //     drawLine(cloth_points_coordinates[i][x_max - 1], cloth_points_coordinates[i+2][x_max - 1]);
    //     drawLine(cloth_points_coordinates[i][x_max - 2], cloth_points_coordinates[i+2][x_max - 2]);
    // }
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
    calculateForce(x_max, y_max, mass, cloth_points_forces, cloth_points_coordinates);
    calcNextState(x_max, y_max, mass, dt, cloth_points_coordinates, cloth_points_velocities, cloth_points_forces);
}

void keyPressed(int key) {
    // See http://www.glfw.org/docs/latest/group__keys.html for key codes
    if (key == GLFW_KEY_SPACE)
        paused = !paused;
}

int main(int argc, char **argv) {
#ifdef HORIZONTAL_CLOTH
    for(int i = 0; i < y_max; i++) {
        for(int j = 0; j < x_max; j++) {
            cloth_points_coordinates[i][j][0] = ((double) j) / x_max;
            cloth_points_coordinates[i][j][1] = 1.0;
            cloth_points_coordinates[i][j][2] = ((double) i) / y_max;
        }
    }
#endif
#ifdef VERTICAL_CLOTH
    for(int i = 0; i < y_max; i++) {
        for(int j = 0; j < x_max; j++) {
            cloth_points_coordinates[i][j][0] = ((double) j) / x_max;
            cloth_points_coordinates[i][j][1] = ((double) i) / y_max;
        }
    }
#endif
    mass = 1.0 / (x_max * y_max);
    auto distance = [](int i1, int j1, int i2, int j2) {
            return sqrt(
                pow(cloth_points_coordinates[i1][j1].x() - cloth_points_coordinates[i2][j2].x(), 2.) + 
                pow(cloth_points_coordinates[i1][j1].y() - cloth_points_coordinates[i2][j2].y(), 2.) +
                pow(cloth_points_coordinates[i1][j1].z() - cloth_points_coordinates[i2][j2].z(), 2.)
            );
    };
    spring_rest_length_x = distance(0, 1, 0, 0);
    spring_rest_length_y = distance(1, 0, 0, 0);
    spring_rest_length_d = distance(1, 1, 0, 0);
    spring_rest_length_x_two_hops = distance(0, 2, 0, 0);
    spring_rest_length_y_two_hops = distance(2, 0, 0, 0);
    // cout << "spring_rest_length: " << spring_rest_length_x << " " << spring_rest_length_y << " " << spring_rest_length_d << " " << spring_rest_length_x_two_hops << " " << spring_rest_length_y_two_hops << std::endl;

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
