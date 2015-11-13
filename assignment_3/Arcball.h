#ifndef ARCBALL_H_
#define ARCBALL_H_

#include <algorithm>

/** Arcball related functions **/
namespace ArcBall {
  /** Compute z given ndc x, y coords **/
  double compute_z(double x, double y) {
    double xx = x*x;
    double yy = y*y;
    return (xx + yy <= 1) ? sqrt(1 - xx - yy) : 0;
  }

  /** Find norm of (x,y,z) vector **/
  constexpr double norm(double x, double y, double z) {
    return sqrt(x*x+y*y+z*z);
  }

  /** Compute angle of rotation given old and new ndc coords **/
  double compute_theta(double xnew, double ynew, double znew,
                      double xold, double yold, double zold) {
    return acos(std::min(double(1),
                         (xnew * xold + ynew * yold + znew * zold) /
                           (norm(xnew, ynew, znew) * norm(xold, yold, zold))));
  }

  /** Set unit vector of axis of rotation **/
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

  /** Compute rotation quaternion given ndc coords **/
  Quaternion compute_rotation_quaternion(double xc, double yc, double xs, double ys) {
    auto zc = compute_z(xc, yc);
    auto zs = compute_z(xs, ys);
    auto theta = compute_theta(xc, yc, zc, xs, ys, zs);
    double ux, uy, uz;
    setU(ux, uy, uz, xc, yc, zc, xs, ys, zs);
    auto to2 = theta/2;
    return Quaternion(cos(to2), ux * sin(to2), uy * sin(to2), uz * sin(to2));
  }

  /** Compute rotation quaternion given screen coords, screen width and height **/
  Quaternion compute_rotation_quaternion(int xc, int yc, int xs, int ys,
                                         int width, int height) {
    auto xcd = static_cast<double>(xc);
    auto ycd = static_cast<double>(yc);
    auto xsd = static_cast<double>(xs);
    auto ysd = static_cast<double>(ys);
    auto widthd = static_cast<double>(width);
    auto heightd = static_cast<double>(height);

    auto xc_ndc = (xcd - widthd/2) / (widthd/2);
    auto yc_ndc = (ycd - heightd/2) / (heightd/2);
    auto xs_ndc = (xsd - widthd/2) / (widthd/2);
    auto ys_ndc = (ysd - heightd/2) / (heightd/2);

    return compute_rotation_quaternion(xc_ndc, -yc_ndc, xs_ndc, -ys_ndc);
  }

}  // namespace ArcBall

#endif  // ARCBALL_H_
