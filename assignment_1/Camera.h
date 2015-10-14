#ifndef CAMERA_H_
#define CAMERA_H_

#include "Rotation.h"
#include "Translation.h"
#include "Perspective.h"
#include <Eigen/Dense>

class Camera {
private:
  TranslationD position;
  RotationD orientation;
  Perspective perspective;

public:
  Eigen::MatrixXd transform;
  Eigen::MatrixXd inverseTransform;
  Eigen::MatrixXd perspective_projection_matrix;
  
  Camera(TranslationD _position, RotationD _orientation, Perspective _perspective)
    : position{_position},
      orientation{_orientation},
      perspective{_perspective},
      inverseTransform{4,4},
      perspective_projection_matrix{4,4} {
    using namespace Eigen;

    transform = position.matrix * orientation.matrix;
    inverseTransform = transform.inverse();
    
    auto n = perspective.near;
    auto f = perspective.far;
    auto t = perspective.top;
    auto b = perspective.bottom;
    auto l = perspective.left;
    auto r = perspective.right;

    perspective_projection_matrix <<
      2 * n / (r - l), 0, (r + l) / (r - l), 0,
      0, 2 * n / (t - b), (t + b) / (t - b), 0,
      0, 0, -(f + n) / (f - n), -2 * f * n / (f - n),
      0, 0, -1, 0;
  }
};

#endif // CAMERA_H_
