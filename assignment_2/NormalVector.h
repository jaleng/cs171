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
    // Normalize Vector
    float magnitude = sqrt(x * x + y * y + z * z);
    x /= magnitude;
    y /= magnitude;
    z /= magnitude;
  };
};

#endif // _NORMALVECTOR_H_
