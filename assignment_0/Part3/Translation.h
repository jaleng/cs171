#ifndef TRANSLATION_H_
#define TRANSLATION_H_

#include <Eigen/Dense>

using namespace Eigen;

/**
 * Translation transformation
 */
template<typename T>
class TranslationGeneral {
public:
  MatrixXd matrix;
  explicit TranslationGeneral(T tx, T ty, T tz)
    : matrix{4, 4} {
    translation_vals_to_matrix(matrix, tx, ty, tz);
  }; 

  /** Make a transformation matrix from translation parameters. */
  static void translation_vals_to_matrix(MatrixXd& matrix, T tx, T ty, T tz) {
    matrix <<
      1, 0, 0, tx,
      0, 1, 0, ty,
      0, 0, 1, tz,
      0, 0, 0,  1;
  }
};

using TranslationD = TranslationGeneral<double>;

#endif // TRANSLATION_H_
