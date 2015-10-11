#ifndef ROTATION_H_
#define ROTATION_H_

#include <Eigen/Dense>

using Eigen::MatrixXd;

/**
 * Rotation transformation
 */
template<typename T>
class RotationGeneral {
public:
  MatrixXd matrix;
  explicit RotationGeneral(T rx, T ry, T rz, T theta)
    : matrix{4, 4} {
    rotate_vals_to_matrix(matrix, rx, ry, rz, theta);
  }; 

  /** Make a transformation matrix from rotation parameters. */
  static void rotation_vals_to_matrix(MatrixXd& matrix, T rx, T ry, T rz, T theta) {
    T ct = cos(theta);
    T st = sin(theta);
    T rx2 = rx * rx;
    T ry2 = ry * ry;
    T rz2 = rz * rz;
    T rx_ry = rx * ry;
    T rx_rz = rx * rz;
    T ry_rz = ry * rz;
    T omct = 1 - ct; // 1 - cos(theta);
    
    matrix <<
      rx2  + (1 - rx2) * ct, rx_ry * omct - rz * st, rx_rz * omct + ry * st, 0,
      rx_ry * omct + rz * st, ry2 + (1 - ry2) * ct, ry_rz * omct - rx * st, 0,
      rx_rz * omct - ry * st, ry_rz * omct + rx * st, rz2 + (1 - rz2) * ct, 0,
      0, 0, 0, 1;
  };
};

using RotationD = RotationGeneral<double>;

#endif // ROTATION_H_
