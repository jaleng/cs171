#ifndef OBJECTDATA_H_
#define OBJECTDATA_H_

#include <vector>
#include "Triple.h"
#include "Face.h"

/**
 * Hold vertex and face information for an object.
 */
class ObjectData {
 public:
  std::vector<Triple> vertices;
  std::vector<Triple> normals;
  std::vector<Face> faces;
  ObjectData() : vertices{}, normals{}, faces{} {}
  explicit ObjectData(std::vector<Triple> _vertices,
                      std::vector<Triple> _normals,
                      std::vector<Face> _faces)
    : vertices{_vertices}, normals{_normals}, faces{_faces} {}
};

#endif // OBJECTDATA_H_
