#include "common.hpp"
#include <bits/stdc++.h>

#ifdef SYMPLECTIC_EULER
void calcNextState
(
    int& x_max, int& y_max, double& mass, float& dt,
    std::vector<std::vector<vec3>>& cloth_points_coordinates,
    std::vector<std::vector<vec3>>& cloth_points_velocities,
    std::vector<std::vector<vec3>>& cloth_points_forces
) {
    std::vector<std::vector<vec3>> cloth_points_accelerations(y_max, std::vector<vec3>(x_max, vec3(0., 0., 0.)));
    for(int i = 0; i < y_max; i++) {
        for(int j = 0; j < x_max; j++) {
            cloth_points_accelerations[i][j] = cloth_points_forces[i][j] / mass;
        }
    }
    for(int i = 0; i < y_max; i++) {
        for(int j = 0; j < x_max; j++) {
            cloth_points_velocities[i][j] = cloth_points_velocities[i][j] + cloth_points_accelerations[i][j] * dt;
        }
    }
    for(int i = 0; i < y_max; i++) {
        for(int j = 0; j < x_max; j++) {
            cloth_points_coordinates[i][j] = cloth_points_coordinates[i][j] + cloth_points_velocities[i][j] * dt;
        }
    }
}
#endif