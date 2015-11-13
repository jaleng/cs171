#ifndef TRANSFORMS_H_
#define TRANSFORMS_H_

enum class TransformType {TRANSLATION, ROTATION, SCALING};

struct Transform {
  TransformType transform_type;
  union {
    float translation[3];
    float rotation[3];
    float scaling[3];
  };
  float rotation_angle;

  // ctor
  Transform(TransformType tt, float x, float y, float z, float angle=0.0)
  : transform_type{tt},
    translation{x, y, z},
    rotation_angle{angle}
  {
    /* switch(tt) { */
    /* case TransformType::TRANSLATION: */
    /*   translation = {x, y, z}; */
    /*   break; */
    /* case TransformType::ROTATION: */
    /*   rotation = {x, y, z}; */
    /*   rotation_angle = angle; */
    /*   break; */
    /* case TransformType::SCALING: */
    /*   scaling = {x, y, z}; */
    /*   break; */
    /* } */
  }

};

#endif  // TRANSFORMS_H_
