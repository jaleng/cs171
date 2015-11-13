#ifndef _OBJECTCOPYINFO_H_
#define _OBJECTCOPYINFO_H_

#include <string>
#include <vector>

/**
 * Hold info for a copy of an object (the object identifier and transform).
 */
class ObjectCopyInfo {
 public:
  std::string obj_id;

  float ambient_reflectance[3];
  float diffuse_reflectance[3];
  float specular_reflectance[3];
  float shininess;

  std::vector<Transform> transforms;

  ObjectCopyInfo(std::string _obj_id,
                 float amb_r, float amb_g, float amb_b,
                 float dif_r, float dif_g, float dif_b,
                 float spc_r, float spc_g, float spc_b,
                 float _shininess,
                 std::vector<Transform> _transforms)
    : obj_id{_obj_id},
      ambient_reflectance{amb_r, amb_g, amb_b},
      diffuse_reflectance{dif_r, dif_g, dif_b},
      specular_reflectance{spc_r, spc_g, spc_g},
      shininess{_shininess},
      transforms{_transforms} {};
};

#endif  // _OBJECTCOPYINFO_H_
