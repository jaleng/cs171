#ifndef ARCBALL_H_
#define ARCBALL_H_

#include <algorithm>

namespace ArcBall {
  float compute_z(float x, float y) {
    float xx = x*x;
    float yy = y*y;
    return (xx + yy <= 1) ? sqrt(1 - xx - yy) : 0;
  }

  constexpr float norm(float x, float y, float z) {
    return sqrt(x*x+y*y+z*z);
  }

  float compute_theta(float xnew, float ynew, float znew,
                      float xold, float yold, float zold) {
    return acos(std::min(float(1),
                         (xnew * xold + ynew * yold + znew * zold) /
                           (norm(xnew, ynew, znew) * norm(xold, yold, zold))));
  }

  void setU(float& ux, float& uy, float& uz,
            float xnew, float ynew, float znew,
            float xold, float yold, float zold) {
    ux = yold * znew - ynew * zold;
    uy = zold * xnew - znew * xold;
    uz = xold * ynew - xnew * yold;

    auto size = norm(ux, uy, uz);
    ux /= size;
    uy /= size;
    uz /= size;
  }

  Quaternion compute_rotation_quaternion(float xc, float yc, float xs, float ys) {
    auto zc = compute_z(xc, yc);
    auto zs = compute_z(xs, ys);
    auto theta = compute_theta(xc, yc, zc, xs, ys, zs);
    float ux, uy, uz;
    setU(ux, uy, uz, xc, yc, zc, xs, ys, zs);
    auto to2 = theta/2;
    return Quaternion(cos(to2), ux * sin(to2), uy * sin(to2), uz * sin(to2));
  }

  Quaternion compute_rotation_quaternion(int xc, int yc, int xs, int ys,
                                         int width, int height) {
    float xcf = (float(xc) - float(width)/2) / (float(width)/2);
    float ycf = (float(yc) - float(height)/2) / (float(height)/2);
    float xsf = (float(xs) - float(width)/2) / (float(width)/2);
    float ysf = (float(ys) - float(height)/2) / (float(height)/2);

    return compute_rotation_quaternion(xcf, -ycf, xsf, -ysf);
  }

}  // namespace ArcBall


#endif  // ARCBALL_H_
