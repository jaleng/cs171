#include <string>
#include <Eigen/Dense>

/**
 * Hold info for a copy of an object (the object identifier and transform).
 */
class ObjectCopyInfo {
public:
  std::string obj_id;
  Eigen::MatrixXd transform;
  ObjectCopyInfo(std::string _obj_id, Eigen::MatrixXd _transform)
    : obj_id{_obj_id}, transform{_transform} {};
};
