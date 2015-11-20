#ifndef POINTLIGHT_H_
#define POINTLIGHT_H_

/** Class for info of a point light **/
struct PointLight {
  double position[4];
  double color[3];
  double attenuation_k;

  // ctor
  PointLight(double x, double y, double z,
             double r, double g, double b,
             double k)
  : position {x, y, z, 1},
    color {r, g, b},
    attenuation_k {k} {}
};

#endif  // POINTLIGHT_H_
