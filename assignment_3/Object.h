#ifndef OBJECT_H_
#define OBJECT_H_

#include <vector>

#include "Triple.h"
#include "Transforms.h"

struct Object {
  std::string object_id;
  std::vector<Triple> vertex_buffer;
  std::vector<Triple> normal_buffer;
  std::vector<Transform> transforms;
  double ambient_reflect[3];
  double diffuse_reflect[3];
  double specular_reflect[3];
  double shininess;
};

#endif  // OBJECT_H_
