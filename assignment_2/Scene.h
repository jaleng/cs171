//Scene.h
#ifndef _SCENE_H_
#define _SCENE_H_

#include <vector>

#include "Camera.h"
#include "Object.h"
#include "ObjectCopy.h"

class Scene {
 public:
  Camera camera;
  std::vector<Light> lights;
  std::vector<Object> objects;
  std::vector<ObjectCopy> object_copies;
}

#endif  // _SCENE_H_
