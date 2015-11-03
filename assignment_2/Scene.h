//Scene.h
#ifndef _SCENE_H_
#define _SCENE_H_

#include <vector>
#include <string>
#include <map>
#include <memory>

#include "Camera.h"
#include "ObjectCopy.h"
#include "ObjectData.h"
#include "Light.h"

class Scene {
 public:
  std::unique_ptr<Camera> camera_up;
  std::unique_ptr<std::vector<Light>> lights_up;
  std::unique_ptr<std::map<std::string, ObjectData>> objid_to_data_up;
  std::unique_ptr<std::vector<ObjectCopy>> object_copies_up;

  Scene(
        std::unique_ptr<Camera> _camera_up,
        std::unique_ptr<std::vector<Light>> _lights_up,
        std::unique_ptr<std::map<std::string, ObjectData>> _objid_to_data_up,
        std::unique_ptr<std::vector<ObjectCopy>> _object_copies_up
        )
    : camera_up{std::move(_camera_up)},
      lights_up{std::move(_lights_up)},
      objid_to_data_up{std::move(_objid_to_data_up)},
      object_copies_up{std::move(_object_copies_up)} {}
};

#endif  // _SCENE_H_
