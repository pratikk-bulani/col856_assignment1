#include "common.hpp"
#include <bits/stdc++.h>

using namespace std;

Eigen::Vector3d gravity(0., -9.8, 0.);
void gravitationalForce(double& mass, Eigen::VectorXd &point_forces) {
    point_forces.reshaped(3, point_forces.rows() / 3).colwise() += gravity * mass;
}

void springForce(Eigen::VectorXd &point_coordinates, Eigen::VectorXd &point_forces, vector<spring> &springs) {
    double distance, theta, phi, force; Eigen::Vector3d delta, force_components;
    for(spring &s : springs) {
        delta = point_coordinates(Eigen::seqN(s.eigen_index_2, 3)) - point_coordinates(Eigen::seqN(s.eigen_index_1, 3));
        distance = delta.norm();
        phi = atan2(sqrt(delta(0)*delta(0) + delta(1)*delta(1)), delta(2));
        theta = atan2(delta(1), delta(0));
        force = s.k * (distance - s.rest_length);

        force_components = force * Eigen::Vector3d(sin(phi) * cos(theta), sin(phi) * sin(theta), cos(phi));
        point_forces(Eigen::seqN(s.eigen_index_1, 3)) += force_components;
        point_forces(Eigen::seqN(s.eigen_index_2, 3)) += -force_components;
    }
}

void calculateForce(double& mass, Eigen::VectorXd &point_forces, Eigen::VectorXd &point_coordinates, vector<spring> &springs) {
    point_forces.setZero();
    gravitationalForce(mass, point_forces);
    springForce(point_coordinates, point_forces, springs);
}

void springJacobianWRTx(vector<spring> &springs, Eigen::MatrixXd &J_x, Eigen::VectorXd &point_coordinates) {
    double dFxdx1, dFxdy1, dFxdz1, dFydx1, dFydy1, dFydz1, dFzdx1, dFzdy1, dFzdz1, tau_square_root, tau; Eigen::Vector3d delta;
    for(spring &s : springs) {
        delta = point_coordinates(Eigen::seqN(s.eigen_index_2, 3)) - point_coordinates(Eigen::seqN(s.eigen_index_1, 3));
        tau_square_root = delta.norm();
        tau = pow(tau_square_root, 2);

        dFxdx1 = -s.k * (1 - s.rest_length / tau_square_root * (1 - delta(0) * delta(0) / tau));
        dFxdy1 = -s.k * s.rest_length * delta(0) * delta(1) / (tau * tau_square_root);
        dFxdz1 = -s.k * s.rest_length * delta(0) * delta(2) / (tau * tau_square_root);
        dFydx1 = dFxdy1;
        dFydy1 = -s.k * (1 - s.rest_length / tau_square_root * (1 - delta(1) * delta(1) / tau));
        dFydz1 = -s.k * s.rest_length * delta(1) * delta(2) / (tau * tau_square_root);
        dFzdx1 = dFxdz1;
        dFzdy1 = dFydz1;
        dFzdz1 = -s.k * (1 - s.rest_length / tau_square_root * (1 - delta(2) * delta(2) / tau));

        // Derivatives of F_12_x
        J_x(s.eigen_index_1 + 0, s.eigen_index_1 + 0) += dFxdx1;
        J_x(s.eigen_index_1 + 0, s.eigen_index_2 + 0) += -dFxdx1;
        J_x(s.eigen_index_1 + 0, s.eigen_index_1 + 1) += dFxdy1;
        J_x(s.eigen_index_1 + 0, s.eigen_index_2 + 1) += -dFxdy1;
        J_x(s.eigen_index_1 + 0, s.eigen_index_1 + 2) += dFxdz1;
        J_x(s.eigen_index_1 + 0, s.eigen_index_2 + 2) += -dFxdz1;
        // Derivatives of F_21_x
        J_x(s.eigen_index_2 + 0, s.eigen_index_1 + 0) += -dFxdx1;
        J_x(s.eigen_index_2 + 0, s.eigen_index_2 + 0) += dFxdx1;
        J_x(s.eigen_index_2 + 0, s.eigen_index_1 + 1) += -dFxdy1;
        J_x(s.eigen_index_2 + 0, s.eigen_index_2 + 1) += dFxdy1;
        J_x(s.eigen_index_2 + 0, s.eigen_index_1 + 2) += -dFxdz1;
        J_x(s.eigen_index_2 + 0, s.eigen_index_2 + 2) += dFxdz1;

        // Derivatives of F_12_y
        J_x(s.eigen_index_1 + 1, s.eigen_index_1 + 0) += dFydx1;
        J_x(s.eigen_index_1 + 1, s.eigen_index_2 + 0) += -dFydx1;
        J_x(s.eigen_index_1 + 1, s.eigen_index_1 + 1) += dFydy1;
        J_x(s.eigen_index_1 + 1, s.eigen_index_2 + 1) += -dFydy1;
        J_x(s.eigen_index_1 + 1, s.eigen_index_1 + 2) += dFydz1;
        J_x(s.eigen_index_1 + 1, s.eigen_index_2 + 2) += -dFydz1;
        // Derivatives of F_21_y
        J_x(s.eigen_index_2 + 1, s.eigen_index_1 + 0) += -dFydx1;
        J_x(s.eigen_index_2 + 1, s.eigen_index_2 + 0) += dFydx1;
        J_x(s.eigen_index_2 + 1, s.eigen_index_1 + 1) += -dFydy1;
        J_x(s.eigen_index_2 + 1, s.eigen_index_2 + 1) += dFydy1;
        J_x(s.eigen_index_2 + 1, s.eigen_index_1 + 2) += -dFydz1;
        J_x(s.eigen_index_2 + 1, s.eigen_index_2 + 2) += dFydz1;

        // Derivatives of F_12_z
        J_x(s.eigen_index_1 + 2, s.eigen_index_1 + 0) += dFzdx1;
        J_x(s.eigen_index_1 + 2, s.eigen_index_2 + 0) += -dFzdx1;
        J_x(s.eigen_index_1 + 2, s.eigen_index_1 + 1) += dFzdy1;
        J_x(s.eigen_index_1 + 2, s.eigen_index_2 + 1) += -dFzdy1;
        J_x(s.eigen_index_1 + 2, s.eigen_index_1 + 2) += dFzdz1;
        J_x(s.eigen_index_1 + 2, s.eigen_index_2 + 2) += -dFzdz1;
        // Derivatives of F_21_z
        J_x(s.eigen_index_2 + 2, s.eigen_index_1 + 0) += -dFzdx1;
        J_x(s.eigen_index_2 + 2, s.eigen_index_2 + 0) += dFzdx1;
        J_x(s.eigen_index_2 + 2, s.eigen_index_1 + 1) += -dFzdy1;
        J_x(s.eigen_index_2 + 2, s.eigen_index_2 + 1) += dFzdy1;
        J_x(s.eigen_index_2 + 2, s.eigen_index_1 + 2) += -dFzdz1;
        J_x(s.eigen_index_2 + 2, s.eigen_index_2 + 2) += dFzdz1;
    }
}

void calculateJacobian(vector<spring> &springs, Eigen::MatrixXd &J_x, Eigen::VectorXd &point_coordinates) {
    J_x.setZero();
    springJacobianWRTx(springs, J_x, point_coordinates);
}