#ifndef _NORMALVECTOR_H_
#define _NORMALVECTOR_H_

#include <Eigen/Dense>

class NormalVector {
 public:
  float x;
  float y;
  float z;

  explicit NormalVector(float _x, float _y, float _z)
    : x{_x}, y{_y}, z{_z} {
    normalize();
  };

  /** Normalize the vector */
  void normalize() {
    float magnitude = sqrt(x * x + y * y + z * z);
    x /= magnitude;
    y /= magnitude;
    z /= magnitude;
  }

  Eigen::Vector3d matrix() const {
    Eigen::Vector3d m;
    m << x, y, z;
    return m;
  }

  static NormalVector transform_normal(NormalVector n, Eigen::MatrixXd t) {
    using Eigen::MatrixXd;
    MatrixXd normal_matrix(4, 1);
    normal_matrix << n.x, n.y, n.z, 1;
    MatrixXd temp(4, 1);
    temp = t * normal_matrix;
    float W = temp(3, 0);
    float x, y, z;
    x = temp(0, 0) / W;
    y = temp(1, 0) / W;
    z = temp(2, 0) / W;
    return NormalVector(x, y, z);
  }
};

#endif // _NORMALVECTOR_H_
