//  Light.h
#ifndef _LIGHT_H_
#define _LIGHT_H_

class Light {
 public:
  /** Coordinates of point light **/
  double x, y, z;

  /** Red, green, and blue values of the point light **/
  double r, g, b;

  /** Attenuation value of the point light **/
  double k;

  explicit Light(double _x, double _y, double _z,
                 double _r, double _g, double _b,
                 double k)
    : x{_x}, y{_y}, z{_z}, r{_r}, g{_g}, b{_b} {}
}

#endif // _LIGHT_H_
