#ifndef QUATERNION_H_
#define QUATERNION_H_

#include <array>
#include <cassert>
#include "Transforms.h"

/** Class for quaternions **/
class Quaternion {
 public:
  // quaternion = (s, (x,y,z))
  double s, x, y, z;

  //// ctors
  // Default construction makes a quaternion identity
  Quaternion() : s{1}, x{0}, y{0}, z{0} {}
  Quaternion(double _s, double _x, double _y, double _z)
    : s{_s}, x{_x}, y{_y}, z{_z} {
  }

  Quaternion(double rx, double ry, double rz,
             double theta, bool rotation_param)
    : s{cos(theta / 2)},
      x{rx * sin(theta / 2)},
      y{ry * sin(theta / 2)},
      z{rz * sin(theta / 2)}
      {
        assert(rotation_param);
        normalize();
      }

  void normalize() {
    auto denom = sqrt(s*s+x*x+y*y+z*z);
    s /= denom;
    x /= denom;
    y /= denom;
    z /= denom;
  }

  //// Operators
  /** Quaternion multiply-assign **/
  void operator*=(const Quaternion& q) {
    double new_s = s * q.s - this->vdot(q);
    double new_x = s * q.x + q.s * x + (y * q.z - z * q.y);
    double new_y = s * q.y + q.s * y + (z * q.x - x * q.z);
    double new_z = s * q.z + q.s * z + (x * q.y - y * q.x);

    s = new_s;
    x = new_x;
    y = new_y;
    z = new_z;
  }

  /** Quaternion multiply **/
  Quaternion operator*(const Quaternion& q) {
    Quaternion copy = *this;
    copy *= q;
    return copy;
  }

  void operator+=(const Quaternion& q) {
    s += q.s;
    x += q.x;
    y += q.y;
    z += q.z;
  }

  Quaternion operator+(const Quaternion& q) const {
    Quaternion res = *this;
    res += q;
    return res;
  }

  //// Misc. utility
  /** Take the dot product of the (x,y,z) vectors **/
  double vdot(const Quaternion& q) const {
    return x * q.x + y * q.y + z* q.z;
  }

  double dot(const Quaternion& q) const {
    return s * q.s + x * q.x + y * q.y + z* q.z;
  }

  /** Get Rotation **/
  Transform getRotation() const {
    double denom = sqrt(1 - s*s);
    double rx = x / denom;
    double ry = y / denom;
    double rz = z / denom;
    double theta = 2 * acos(s);
    return Transform(TransformType::ROTATION, x, y, z, theta);
  }

  /** Get rotation matrix from quaternion **/
  std::array<double, 16> getRotationMatrix() {
    std::array<double, 16> m;
    // 1st col
    m[0] = 1 - 2*y*y - 2*z*z;
    m[1] = 2 * (x*y + z*s);
    m[2] = 2 * (x*z - y*s);
    m[3] = 0;

    // 2nd col
    m[4] = 2 * (x*y - z*s);
    m[5] = 1 - 2*x*x - 2*z*z;
    m[6] = 2 * (y*z + x*s);
    m[7] = 0;

    // 3rd col
    m[8] = 2 * (x*z + y*s);
    m[9] = 2 * (y*z - x*s);
    m[10] = 1 - 2*x*x - 2*y*y;
    m[11] = 0;

    // 4th col
    m[12] = 0;
    m[13] = 0;
    m[14] = 0;
    m[15] = 1;

    return std::move(m);
  }

  //// Static functions
  /** Quaternion identity = (1, (0,0,0)) **/
  static Quaternion identity() {
    return Quaternion();
  }
};

Quaternion operator*(double scale, const Quaternion& q) {
  return Quaternion(scale*q.s, scale*q.x, scale*q.y, scale*q.z);
}

#endif  // QUATERNION_H_
