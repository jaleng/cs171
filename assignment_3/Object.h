#ifndef OBJECT_H_
#define OBJECT_H_

#include <vector>
#include <string>

#include "Triple.h"
#include "Transforms.h"

/* Class to store an object's vertex buffer, normal buffer,
 * transforms, and material (light) parameters 
 */
struct Object {
  /** Name of the object (ie what kind of object is this a copy of?) **/
  std::string object_id;
  /** Vertex buffer, opengl friendly **/
  std::vector<Triple> vertex_buffer;
  /** Normal buffer, opengl friendly **/
  std::vector<Triple> normal_buffer;
  /** Transforms, in order that they should be applied in **/
  std::vector<Transform> transforms;

  //// Material (light reflection) parameters
  /** Ambient reflectance **/
  double ambient_reflect[3];
  /** Diffuse reflectance **/
  double diffuse_reflect[3];
  /** Specular reflectance **/
  double specular_reflect[3];
  /** Phong shininess parameter **/
  double shininess;
};

#endif  // OBJECT_H_
