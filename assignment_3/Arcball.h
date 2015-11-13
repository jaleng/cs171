#ifndef ARCBALL_H_
#define ARCBALL_H_

#include <algorithm>

namespace ArcBall {
  double compute_z(double x, double y) {
    double xx = x*x;
    double yy = y*y;
    return (xx + yy <= 1) ? sqrt(1 - xx - yy) : 0;
  }

  constexpr double norm(double x, double y, double z) {
    return sqrt(x*x+y*y+z*z);
  }

  double compute_theta(double xnew, double ynew, double znew,
                      double xold, double yold, double zold) {
    return acos(std::min(double(1),
                         (xnew * xold + ynew * yold + znew * zold) /
                           (norm(xnew, ynew, znew) * norm(xold, yold, zold))));
  }

  void setU(double& ux, double& uy, double& uz,
            double xnew, double ynew, double znew,
            double xold, double yold, double zold) {
    ux = yold * znew - ynew * zold;
    uy = zold * xnew - znew * xold;
    uz = xold * ynew - xnew * yold;

    auto size = norm(ux, uy, uz);
    ux /= size;
    uy /= size;
    uz /= size;
  }

  Quaternion compute_rotation_quaternion(double xc, double yc, double xs, double ys) {
    auto zc = compute_z(xc, yc);
    auto zs = compute_z(xs, ys);
    auto theta = compute_theta(xc, yc, zc, xs, ys, zs);
    double ux, uy, uz;
    setU(ux, uy, uz, xc, yc, zc, xs, ys, zs);
    auto to2 = theta/2;
    return Quaternion(cos(to2), ux * sin(to2), uy * sin(to2), uz * sin(to2));
  }

  Quaternion compute_rotation_quaternion(int xc, int yc, int xs, int ys,
                                         int width, int height) {
    double xcf = (double(xc) - double(width)/2) / (double(width)/2);
    double ycf = (double(yc) - double(height)/2) / (double(height)/2);
    double xsf = (double(xs) - double(width)/2) / (double(width)/2);
    double ysf = (double(ys) - double(height)/2) / (double(height)/2);

    return compute_rotation_quaternion(xcf, -ycf, xsf, -ysf);
  }

}  // namespace ArcBall


#endif  // ARCBALL_H_
