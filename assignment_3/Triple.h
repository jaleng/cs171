#ifndef TRIPLE_H_
#define TRIPLE_H_

struct Triple {
  float x;
  float y;
  float z;

  // ctor
  Triple(float _x, float _y, float _z)
  : x{_x}, y{_y}, z{_z} {}
};

#endif  // TRIPLE_H_
