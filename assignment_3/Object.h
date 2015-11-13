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
  float ambient_reflect[3];
  float diffuse_reflect[3];
  float specular_reflect[3];
  float shininess;
};

#endif  // OBJECT_H_
