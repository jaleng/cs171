//Object.h
#ifndef _OBJECT_H_
#define _OBJECT_H_

#include <vector>
#include <string>

#include "Vertex.h"
#include "NormalVector.h"
#include "Face.h"

class Object {
 public:
  std::string id;
  std::vector<Vertex> vertices;
  std::vector<NormalVector> normals;
  std::vector<Face> faces;
};

#endif // _OBJECT_H_
