#include "common.hpp"
#include <bits/stdc++.h>

vec3 gravity(0., -9.8, 0.);
void gravitationalForce(int& x_max, int& y_max, double& mass, std::vector<std::vector<vec3>>& cloth_points_forces) {
    for(int i = 0; i < y_max; i++) {
        for(int j = 0; j < x_max; j++) {
            cloth_points_forces[i][j] += gravity * mass;
            // std::cout << cloth_points_forces[i][j].x() << " " << cloth_points_forces[i][j].y() << " " << cloth_points_forces[i][j].z() << " " << mass << std::endl;
        }
    }
}

double spring_constant_x = 100., spring_constant_y = 100., spring_constant_d = 50., spring_constant_x_two_hops = 1., spring_constant_y_two_hops = 1.,
        spring_rest_length_x = 0., spring_rest_length_y = 0., spring_rest_length_d = 0., spring_rest_length_x_two_hops = 0., spring_rest_length_y_two_hops = 0.;
inline std::vector<double> calcThetaPhiDistance(vec3& point1, vec3& point2) {
    double delta_x = point2.x() - point1.x(), delta_y = point2.y() - point1.y(), delta_z = point2.z() - point1.z();
    std::vector<double> result;
    result.push_back(atan2(delta_y, delta_x));
    result.push_back(atan2(sqrt(delta_x*delta_x + delta_y*delta_y), delta_z));
    result.push_back(sqrt(delta_x*delta_x + delta_y*delta_y + delta_z*delta_z));
    return result;
}
inline vec3 calcSpringForce(vec3& point1, vec3& point2, double& spring_rest_length, double& spring_constant) {
    std::vector<double> theta_phi_distance = calcThetaPhiDistance(point1, point2);
    double spring_force = spring_constant * (theta_phi_distance[2] - spring_rest_length);
    return vec3(spring_force * sin(theta_phi_distance[1]) * cos(theta_phi_distance[0]), 
                spring_force * sin(theta_phi_distance[1]) * sin(theta_phi_distance[0]), 
                spring_force * cos(theta_phi_distance[1])
               );
}
inline void updateSpringForce(int i1, int j1, int i2, int j2, double& spring_rest_length, double& spring_constant, std::vector<std::vector<vec3>>& cloth_points_forces, std::vector<std::vector<vec3>>& cloth_points_coordinates) {
    vec3 spring_force = calcSpringForce(cloth_points_coordinates[i1][j1], cloth_points_coordinates[i2][j2], spring_rest_length, spring_constant);
    cloth_points_forces[i1][j1] += spring_force;
    cloth_points_forces[i2][j2] += -spring_force;
}
void springForce(int& x_max, int& y_max, std::vector<std::vector<vec3>>& cloth_points_forces, std::vector<std::vector<vec3>>& cloth_points_coordinates) {
    vec3 spring_force;
    for(int i = 0; i < y_max - 1; i++) {
        for(int j = 0; j < x_max - 1; j++) {
            updateSpringForce(i, j, i+1, j, spring_rest_length_y, spring_constant_y, cloth_points_forces, cloth_points_coordinates);
            updateSpringForce(i, j, i, j+1, spring_rest_length_x, spring_constant_x, cloth_points_forces, cloth_points_coordinates);
            updateSpringForce(i, j, i+1, j+1, spring_rest_length_d, spring_constant_d, cloth_points_forces, cloth_points_coordinates);
        }
    }
    for(int i = y_max - 1; i > 0; i--) {
        for(int j = 0; j < x_max - 1; j++) {
            updateSpringForce(i, j, i-1, j+1, spring_rest_length_d, spring_constant_d, cloth_points_forces, cloth_points_coordinates);
        }
    }
    for(int i = 0; i < x_max - 1; i++) {
        updateSpringForce(y_max - 1, i, y_max - 1, i+1, spring_rest_length_x, spring_constant_x, cloth_points_forces, cloth_points_coordinates);
    }
    for(int i = 0; i < y_max - 1; i++) {
        updateSpringForce(i, x_max - 1, i+1, x_max - 1, spring_rest_length_y, spring_constant_y, cloth_points_forces, cloth_points_coordinates);
    }

    // Two hops away
    for(int i = 0; i < y_max - 2; i++) {
        for(int j = 0; j < x_max - 2; j++) {
            updateSpringForce(i, j, i+2, j, spring_rest_length_y_two_hops, spring_constant_y_two_hops, cloth_points_forces, cloth_points_coordinates);
            updateSpringForce(i, j, i, j+2, spring_rest_length_x_two_hops, spring_constant_x_two_hops, cloth_points_forces, cloth_points_coordinates);
        }
    }
    for(int i = 0; i < x_max - 2; i++) {
        updateSpringForce(y_max - 1, i, y_max - 1, i+2, spring_rest_length_x_two_hops, spring_constant_x_two_hops, cloth_points_forces, cloth_points_coordinates);
        updateSpringForce(y_max - 2, i, y_max - 2, i+2, spring_rest_length_x_two_hops, spring_constant_x_two_hops, cloth_points_forces, cloth_points_coordinates);
    }
    for(int i = 0; i < y_max - 2; i++) {
        updateSpringForce(i, x_max - 1, i+2, x_max - 1, spring_rest_length_y_two_hops, spring_constant_y_two_hops, cloth_points_forces, cloth_points_coordinates);
        updateSpringForce(i, x_max - 2, i+2, x_max - 2, spring_rest_length_y_two_hops, spring_constant_y_two_hops, cloth_points_forces, cloth_points_coordinates);
    }
}

void calculateForce(int& x_max, int& y_max, double& mass, std::vector<std::vector<vec3>>& cloth_points_forces, std::vector<std::vector<vec3>>& cloth_points_coordinates) {
    std::fill(cloth_points_forces.begin(), cloth_points_forces.end(), std::vector<vec3>(x_max, vec3(0., 0., 0.)));
    gravitationalForce(x_max, y_max, mass, cloth_points_forces);
    springForce(x_max, y_max, cloth_points_forces, cloth_points_coordinates);
    // Fixed points
#ifdef HORIZONTAL_CLOTH
    cloth_points_forces[0][0] = cloth_points_forces[0][x_max - 1] = vec3(0., 0., 0.);
#endif
#ifdef VERTICAL_CLOTH
    cloth_points_forces[y_max - 1][0] = cloth_points_forces[y_max - 1][x_max - 1] = vec3(0., 0., 0.);
#endif
}