#ifndef QUATERNION_H_
#define QUATERNION_H_

/** Class for quaternions **/
class Quaternion {
 private:
  // quaternion = (s, (x,y,z))
  double s, x, y, z;

 public:
  //// ctors
  // Default construction makes a quaternion identity
  Quaternion() : s{1}, x{0}, y{0}, z{0} {}
  Quaternion(double _s, double _x, double _y, double _z)
    : s{_s}, x{_x}, y{_y}, z{_z} {}

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

  //// Misc. utility
  /** Take the dot product of the (x,y,z) vectors **/
  double vdot(const Quaternion& q) const {
    return x * q.x + y * q.y + z* q.z;
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

#endif  // QUATERNION_H_
