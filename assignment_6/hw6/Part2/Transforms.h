#ifndef TRANSFORMS_H_
#define TRANSFORMS_H_

/** Object transformation types **/
enum class TransformType {TRANSLATION, ROTATION, SCALING};

struct Transform {
  // Type of this transform (translation, rotation, or scaling)
  TransformType transform_type;

  /** (x,y,z) parameters of the transformation **/
  union {
    double translation[3];
    double rotation[3];
    double scaling[3];
  };

  /** Rotation angle (only valid if this Transform is a rotation) **/
  double rotation_angle;

  // ctor
  /** Construct Transform by passing in the type and parameters **/
  Transform(TransformType tt, double x, double y, double z, double angle=0.0)
  : transform_type{tt},
    translation{x, y, z},
    rotation_angle{angle} {}

  Transform(TransformType tt)
  : transform_type{tt}, translation{0,0,0}, rotation_angle{0} {}

  // mutators
  void set(double x, double y, double z, double angle=0.0) {
    translation[0] = x;
    translation[1] = y;
    translation[2] = z;
    rotation_angle = angle;
  }
};

#endif  // TRANSFORMS_H_
