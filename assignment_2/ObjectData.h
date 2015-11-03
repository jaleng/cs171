#ifndef OBJECTDATA_H_
#define OBJECTDATA_H_

#include <vector>
#include "Vertex.h"
#include "Face.h"
#include "NormalVector.h"

/**
 * Hold vertex and face information for an object.
 */
class ObjectData {
 public:
  std::vector<Vertex> vertices;
  std::vector<NormalVector> normals;
  std::vector<Face> faces;
  ObjectData() : vertices{}, normals{}, faces{} {}
  explicit ObjectData(std::vector<Vertex> _vertices,
                      std::vector<NormalVector> _normals,
                      std::vector<Face> _faces)
    : vertices{_vertices}, normals{_normals}, faces{_faces} {}
};

#endif // OBJECTDATA_H_
