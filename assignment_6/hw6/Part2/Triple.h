#ifndef TRIPLE_H_
#define TRIPLE_H_

/** Store an x,y,z triple **/
struct Triple {
  double x;
  double y;
  double z;

  // ctor
  Triple(double _x, double _y, double _z)
  : x{_x}, y{_y}, z{_z} {}
};

#endif  // TRIPLE_H_
