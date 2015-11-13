#ifndef TRANSFORMS_H_
#define TRANSFORMS_H_

enum class TransformType {TRANSLATION, ROTATION, SCALING};

struct Transform {
  TransformType transform_type;
  union {
    double translation[3];
    double rotation[3];
    double scaling[3];
  };
  double rotation_angle;

  // ctor
  Transform(TransformType tt, double x, double y, double z, double angle=0.0)
  : transform_type{tt},
    translation{x, y, z},
    rotation_angle{angle}
  {}

};

#endif  // TRANSFORMS_H_
