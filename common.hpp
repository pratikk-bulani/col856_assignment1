#ifndef COMMON_HPP
#define COMMON_HPP

#include <Eigen/Dense>

#if __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif

typedef Eigen::Vector2f vec2;
typedef Eigen::Vector3f vec3;

inline long calcEigenIndex(int i, int j, long cols) {
    return 3*(i*cols + j);
}

struct spring {
    long eigen_index_1, eigen_index_2;
    double k, rest_length;
};

#endif
