#include "common.hpp"
#include <bits/stdc++.h>
#include "forces.hpp"

using namespace std;

#ifdef SYMPLECTIC_EULER
void calcNextState(float &dt, PointGeometry &pg) {
    pg.point_velocities += pg.mass_inverse * pg.point_forces * dt;
    pg.point_coordinates += pg.point_velocities * dt;
}
#endif

#ifdef BACKWARD_EULER_NEWTON
void calcNextState(float &dt, PointGeometry &pg) {
    // Create the Jacobian matrix
    Eigen::MatrixXd J_x = Eigen::MatrixXd::Zero(pg.point_coordinates.rows(), pg.point_coordinates.rows());

    // Fill the Jacobian matrix
    calculateJacobian(pg.springs, J_x, pg.point_coordinates);

    // Calculate the next velocities
    pg.point_velocities += (Eigen::MatrixXd::Identity(pg.point_coordinates.rows(), pg.point_coordinates.rows()) - dt*dt*pg.mass_inverse*J_x).colPivHouseholderQr().solve(dt*pg.mass_inverse*pg.point_forces + dt*dt*pg.mass_inverse*J_x*pg.point_velocities);

    // Calculate the next coordinates
    pg.point_coordinates += dt*pg.point_velocities;
}
#endif