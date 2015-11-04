#ifndef _MATERIAL_H_
#define _MATERIAL_H_

/** Class to hold information about a material (reflectances, shininess). */
class Material{
 public:
  /** Ambient material reflectance **/
  Eigen::MatrixXd ambient_reflectance;
  /** Diffuse material reflectance **/
  Eigen::MatrixXd diffuse_reflectance;
  /** Specular material reflectance **/
  Eigen::MatrixXd specular_reflectance;
  /** Shininess, ie Phong exponent p **/
  double shininess;
};

#endif
