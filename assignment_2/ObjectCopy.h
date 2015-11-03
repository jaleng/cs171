//ObjectCopy.h
#ifndef _OBJECTCOPY_H_
#define _OBJECTCOPY_H_

#include <Eigen/Dense>
#include <vector>
#include <string>

#include "Vertex.h"
#include "NormalVector.h"
#include "Face.h"
#include "Material.h"

class ObjectCopy {
 public:
  /** ID of object this is a copy of **/
  std::string object_id;
  /** Transformed world-space vertices **/
  std::vector<Vertex> vertices;
  /** Transformed world-space normals **/
  std::vector<NormalVector> normals;
  /** Faces of this ObjectCopy **/
  std::vector<Face> faces;

  /** Material of this ObjectCopy **/
  Material material;

  // moved to Material
  /* /\** Ambient material reflectance **\/ */
  /* Eigen::MatrixXd ambient_reflectance; */
  /* /\** Diffuse material reflectance **\/ */
  /* Eigen::MatrixXd diffuse_reflectance; */
  /* /\** Specular material reflectance **\/ */
  /* Eigen::MatrixXd specular_reflectance; */
  /* /\** Shininess, ie Phong exponent p **\/ */
  /* double shininess; */
};

#endif // _OBJECTCOPY_H_ 