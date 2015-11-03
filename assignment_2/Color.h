#ifndef _COLOR_H_
#define _COLOR_H_

#include <Eigen/Dense>

class Color {
 public:
  double red;
  double green;
  double blue;

  Color(double _red, double _green, double _blue)
    : red{_red}, green{_green}, blue{_blue} {}

  Color(const MatrixXd& m) {
    red = m(0);
    green = m(1);
    blue =  m(2);
  }

  Eigen::MatrixXd matrix() const {
    using Eigen::MatrixXd;
    MatrixXd m(3, 1);
    m << red, green, blue;
    return m;
  }
};

#endif // _COLOR_H_
