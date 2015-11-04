#ifndef _SCENEPARSER_H_
#define _SCENEPARSER_H_

#include <fstream>
#include <string>
#include <sstream>
#include <cassert>
#include <memory>
#include <map>
#include <vector>

#include "Camera.h"
#include "Light.h"
#include "Scene.h"
#include "ObjectData.h"
#include "ObjectCopyInfo.h"

namespace SceneParser {
  /** Parse a scene **/
  std::unique_ptr<Scene> parse_scene(std::ifstream& scene_desc_file_stream);

  /** Parse camera parameters and return a Camera unique_ptr. */
  std::unique_ptr<Camera> parse_camera(std::ifstream& file_stream);

  /** Parse lights **/
  std::unique_ptr<std::vector<Light>> parse_lights (std::ifstream& file_stream);

  /** Parse object attributes. */
  std::unique_ptr<std::map<std::string, std::string>>
  parse_obj_to_filename(std::ifstream& file_stream);

  /** Parse object data. */
  std::unique_ptr<std::map<std::string, ObjectData>>
  make_obj_to_data(const std::map<std::string, std::string>* obj_name_to_file_name_p);

  /** Parse object_copy attributes. */
  std::unique_ptr<std::vector<ObjectCopyInfo>>
  parse_obj_copy_info(std::ifstream& file_stream);

  /** Make a vector of ObjectCopys using object data and copy info. **/
  std::unique_ptr<std::vector<ObjectCopy>>
  make_obj_copy_vec(std::vector<ObjectCopyInfo> *objcpy_info_vec_p,
                    std::map<std::string, ObjectData> *objid_to_data_p);
};  // namespace SceneParser

#endif // _SCENEPARSER_H_
