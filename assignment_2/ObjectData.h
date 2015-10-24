#ifndef OBJECTDATA_H_
#define OBJECTDATA_H_

#include <vector>
#include "Vertex.h"
#include "Face.h"

/**
 * Hold vertex and face information for an object.
 */
class ObjectData {
 public:
  std::vector<Vertex> vertices;
  std::vector<Face> faces;
  explicit ObjectData() : vertices{}, faces{} {}
  explicit ObjectData(std::vector<Vertex> _vertices, std::vector<Face> _faces)
    : vertices{_vertices}, faces{_faces} {}
};

#endif // OBJECTDATA_H_
