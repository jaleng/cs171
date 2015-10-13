#ifndef SCALE_H_
#define SCALE_H_

#include <Eigen/Dense>

/**
 * Scaling transformation
 */
template<typename T>
class ScaleGeneral {
public:
  Eigen::MatrixXd matrix;
  explicit ScaleGeneral(T sx, T sy, T sz)
    : matrix{4, 4} {
    scale_vals_to_matrix(matrix, sx, sy, sz);
  }; 

  /** Turn scaling parameters into a transformation matrix. */
  static void scale_vals_to_matrix(Eigen::MatrixXd& matrix, T sx, T sy, T sz) {
    matrix <<
      sx, 0,  0,  0,
      0,  sy, 0,  0,
      0,  0,  sz, 0,
      0,  0,  0,  1;
  };
};

using ScaleD = ScaleGeneral<double>;

#endif // SCALE_H_
