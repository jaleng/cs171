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

  double ambient_reflectance[3];
  double diffuse_reflectance[3];
  double specular_reflectance[3];
  double shininess;

  std::vector<Transform> transforms;

  ObjectCopyInfo(std::string _obj_id,
                 double amb_r, double amb_g, double amb_b,
                 double dif_r, double dif_g, double dif_b,
                 double spc_r, double spc_g, double spc_b,
                 double _shininess,
                 std::vector<Transform> _transforms)
    : obj_id{_obj_id},
      ambient_reflectance{amb_r, amb_g, amb_b},
      diffuse_reflectance{dif_r, dif_g, dif_b},
      specular_reflectance{spc_r, spc_g, spc_g},
      shininess{_shininess},
      transforms{_transforms} {};
};

#endif  // _OBJECTCOPYINFO_H_
