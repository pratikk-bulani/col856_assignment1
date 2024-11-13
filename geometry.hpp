#include "draw.hpp"
#include "common.hpp"
#include <bits/stdc++.h>

using namespace std;

double spring_constant_x = 100., spring_constant_y = 100., spring_constant_d = 50., spring_constant_x_two_hops = 1., spring_constant_y_two_hops = 1.;

class PointGeometry {
private:
    void pointGeometryDrawPoint() {
        for(int i = 0; i < y_max; i++) {
            for(int j = 0; j < x_max; j++) {
                drawPoint(point_coordinates(Eigen::seqN(calcEigenIndex(i, j, x_max), 3)).cast<float>());
            }
        }
    }
    void pointGeometryDrawLine() {
        for(int i = 0; i < y_max - 1; i++) {
            for(int j = 0; j < x_max - 1; j++) {
                drawLine(point_coordinates(Eigen::seqN(calcEigenIndex(i, j, x_max), 3)).cast<float>(), point_coordinates(Eigen::seqN(calcEigenIndex(i+1, j, x_max), 3)).cast<float>());
                drawLine(point_coordinates(Eigen::seqN(calcEigenIndex(i, j, x_max), 3)).cast<float>(), point_coordinates(Eigen::seqN(calcEigenIndex(i, j+1, x_max), 3)).cast<float>());
                drawLine(point_coordinates(Eigen::seqN(calcEigenIndex(i, j, x_max), 3)).cast<float>(), point_coordinates(Eigen::seqN(calcEigenIndex(i+1, j+1, x_max), 3)).cast<float>());
            }
        }
        for(int i = y_max - 1; i > 0; i--) {
            for(int j = 0; j < x_max - 1; j++) {
                drawLine(point_coordinates(Eigen::seqN(calcEigenIndex(i, j, x_max), 3)).cast<float>(), point_coordinates(Eigen::seqN(calcEigenIndex(i-1, j+1, x_max), 3)).cast<float>());
            }
        }
        for(int i = 0; i < x_max - 1; i++) {
            drawLine(point_coordinates(Eigen::seqN(calcEigenIndex(y_max-1, i, x_max), 3)).cast<float>(), point_coordinates(Eigen::seqN(calcEigenIndex(y_max-1, i+1, x_max), 3)).cast<float>());
        }
        for(int i = 0; i < y_max - 1; i++) {
            drawLine(point_coordinates(Eigen::seqN(calcEigenIndex(i, x_max-1, x_max), 3)).cast<float>(), point_coordinates(Eigen::seqN(calcEigenIndex(i+1, x_max-1, x_max), 3)).cast<float>());
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
    void createSpring(int row_index1, int col_index1, int row_index2, int col_index2, int x_max) {
        spring s;
        s.eigen_index_1 = calcEigenIndex(row_index1, col_index1, x_max);
        s.eigen_index_2 = calcEigenIndex(row_index2, col_index2, x_max);
        if(row_index2 == row_index1) {
            if(col_index1 == col_index2) {
                cout << "SEO in row and col index values" << endl;
                exit(0);
            } else if(col_index2 == col_index1 + 1) {
                s.k = spring_constant_x;
            } else if(col_index2 == col_index1 + 2) {
                s.k = spring_constant_x_two_hops;
            } else {
                cout << "SEO in col index values" << endl;
                exit(0);
            }
        }
        else if(row_index2 == row_index1 + 1) {
            if(col_index1 == col_index2) {
                s.k = spring_constant_y;
            } else if(col_index2 == col_index1 + 1) {
                s.k = spring_constant_d;
            } else {
                cout << "SEO in col index values" << endl;
                exit(0);
            }
        } else if(row_index2 == row_index1 + 2) {
            if(col_index1 == col_index2) {
                s.k = spring_constant_y_two_hops;
            } else {
                cout << "SEO in col index values" << endl;
                exit(0);
            }
        } else if(row_index2 == row_index1 - 1 && col_index2 == col_index1 + 1) {
            s.k = spring_constant_d;
        } else {
            cout << "SEO in row index values" << endl;
            exit(0);
        }
        s.rest_length = (this->point_coordinates(Eigen::seqN(s.eigen_index_1, 3)) - this->point_coordinates(Eigen::seqN(s.eigen_index_2, 3))).norm();
        this->springs.push_back(s);
    }
public:
    int x_max, y_max;
    double mass;
    Eigen::VectorXd point_coordinates, point_velocities, point_forces;
    vector<spring> springs;
    vector<long> fixed_points; // Points for which the force should be made zero
    Eigen::MatrixXd mass_inverse;

    PointGeometry(int x_max, int y_max) {
        this->x_max = x_max;
        this->y_max = y_max;
        this->mass = 1.0 / (x_max * y_max);
        this->point_coordinates = Eigen::VectorXd::Zero(x_max * y_max * 3);
        this->point_velocities = Eigen::VectorXd::Zero(x_max * y_max * 3);
        this->point_forces = Eigen::VectorXd::Zero(x_max * y_max * 3);

        #ifdef HORIZONTAL_CLOTH
        for(int i = 0; i < y_max; i++) {
            for(int j = 0; j < x_max; j++) {
                long eigen_index = calcEigenIndex(i, j, x_max);
                point_coordinates(eigen_index + 0) = (x_max - 1) == 0 ? 1. : ((double) j) / (x_max - 1);
                point_coordinates(eigen_index + 1) = 1.0;
                point_coordinates(eigen_index + 2) = (y_max - 1) == 0 ? 1. : ((double) i) / (y_max - 1);
            }
        }
        this->fixed_points.push_back(calcEigenIndex(0, 0, x_max));
        this->fixed_points.push_back(calcEigenIndex(0, x_max - 1, x_max));
        #endif
        #ifdef VERTICAL_CLOTH
        for(int i = 0; i < y_max; i++) {
            for(int j = 0; j < x_max; j++) {
                long eigen_index = calcEigenIndex(i, j, x_max);
                point_coordinates(eigen_index + 0) = (x_max - 1) == 0 ? 1. : ((double) j) / (x_max - 1);
                point_coordinates(eigen_index + 1) = (y_max - 1) == 0 ? 1. : ((double) i) / (y_max - 1);
                point_coordinates(eigen_index + 2) = 0.0;
            }
        }
        this->fixed_points.push_back(calcEigenIndex(y_max - 1, 0, x_max));
        this->fixed_points.push_back(calcEigenIndex(y_max - 1, x_max - 1, x_max));
        #endif

        createSprings();
        createMassInverseMatrix();
    }

    void createSprings() {
        spring s;
        for(int i = 0; i < y_max - 1; i++) {
            for(int j = 0; j < x_max - 1; j++) {
                createSpring(i, j, i+1, j, x_max);
                createSpring(i, j, i, j+1, x_max);
                createSpring(i, j, i+1, j+1, x_max);
            }
        }
        for(int i = y_max - 1; i > 0; i--) {
            for(int j = 0; j < x_max - 1; j++) {
                createSpring(i, j, i-1, j+1, x_max);
            }
        }
        for(int i = 0; i < x_max - 1; i++) {
            createSpring(y_max-1, i, y_max-1, i+1, x_max);
        }
        for(int i = 0; i < y_max - 1; i++) {
            createSpring(i, x_max-1, i+1, x_max-1, x_max);
        }

        // Two hops away
        for(int i = 0; i < y_max - 2; i++) {
            for(int j = 0; j < x_max - 2; j++) {
                createSpring(i, j, i+2, j, x_max);
                createSpring(i, j, i, j+2, x_max);
            }
        }
        for(int i = 0; i < x_max - 2; i++) {
            createSpring(y_max-1, i, y_max-1, i+2, x_max);
            createSpring(y_max-2, i, y_max-2, i+2, x_max);
        }
        for(int i = 0; i < y_max - 2; i++) {
            createSpring(i, x_max-1, i+2, x_max-1, x_max);
            createSpring(i, x_max-2, i+2, x_max-2, x_max);
        }
    }

    void draw() {
        this->pointGeometryDrawPoint();
        this->pointGeometryDrawLine();
    }

    void createMassInverseMatrix() {
        this->mass_inverse = (1 / this->mass) * Eigen::MatrixXd::Identity(point_coordinates.rows(), point_coordinates.rows());
        for(long eigen_index : this->fixed_points) {
            mass_inverse(Eigen::seqN(eigen_index, 3), Eigen::seqN(eigen_index, 3)) = Eigen::Matrix3d::Zero();
        }
    }
};