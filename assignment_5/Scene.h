#ifndef _SCENE_H_
#define _SCENE_H_

#include <vector>
#include <string>
#include <map>
#include <memory>

#include "Camera.h"
#include "Object.h"
#include "ObjectData.h"
#include "PointLight.h"

/** Hold scene info (camera, lights, objects and their transforms) **/
class Scene {
 public:
  // Scene object has ownership of its camera, lights, objects, etc.

  std::unique_ptr<Camera> camera_up;
  std::unique_ptr<std::vector<PointLight>> lights_up;
  std::unique_ptr<std::map<std::string, ObjectData>> objid_to_data_up;
  std::unique_ptr<std::vector<Object>> objects_up;

  //// ctor
  Scene(
        std::unique_ptr<Camera> _camera_up,
        std::unique_ptr<std::vector<PointLight>> _lights_up,
        std::unique_ptr<std::map<std::string, ObjectData>> _objid_to_data_up,
        std::unique_ptr<std::vector<Object>> _objects_up
        )
    : camera_up{std::move(_camera_up)},
      lights_up{std::move(_lights_up)},
      objid_to_data_up{std::move(_objid_to_data_up)},
      objects_up{std::move(_objects_up)} {}
};

#endif  // _SCENE_H_
