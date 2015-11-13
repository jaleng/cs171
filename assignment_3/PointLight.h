#ifndef POINTLIGHT_H_
#define POINTLIGHT_H_

struct PointLight {
  float position[4];
  float color[3];
  float attenuation_k;

  // ctor
  PointLight(float x, float y, float z,
             float r, float g, float b,
             float k)
  : position {x, y, z, 1},
    color {r, g, b},
    attenuation_k {k} {}
};

#endif  // POINTLIGHT_H_
