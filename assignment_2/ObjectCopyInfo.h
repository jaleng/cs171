#ifndef _OBJECTCOPYINFO_H_
#define _OBJECTCOPYINFO_H_

#include <string>
#include <Eigen/Dense>

/**
 * Hold info for a copy of an object (the object identifier and transform).
 */
class ObjectCopyInfo {
 public:
  std::string obj_id;

  Eigen::MatrixXd ambient_reflectance;
  Eigen::MatrixXd diffuse_reflectance;
  Eigen::MatrixXd specular_reflectance;
  double shininess;

  Eigen::MatrixXd transform;

  ObjectCopyInfo(
    std::string _obj_id,
    double amb_r, double amb_g, double amb_b,
    double dif_r, double dif_g, double dif_b,
    double spc_r, double spc_g, double spc_b,
    double shininess,
    Eigen::MatrixXd _transform
    )
    : obj_id{_obj_id},
      ambient_reflectance{3, 1},
      diffuse_reflectance{3, 1},
      specular_reflectance{3, 1},
      shininess{_shininess},
      transform{_transform} {

    ambient_reflectance(0) = amb_r;
    ambient_reflectance(1) = amb_g;
    ambient_reflectance(2) = amb_b;

    diffuse_reflectance(0) = dif_r;
    diffuse_reflectance(1) = dif_g;
    diffuse_reflectance(2) = dif_b;

    specular_reflectance(0) = spc_r;
    specular_reflectance(1) = spc_g;
    specular_reflectance(2) = spc_b;
  };
};

#endif  // _OBJECTCOPYINFO_H_
