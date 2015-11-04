#ifndef VERTEX_H_
#define VERTEX_H_

#include <Eigen/Dense>

/** Hold 3 double coordinates for x,y,z space */
class Vertex {
 public:
  double x;
  double y;
  double z;

  explicit Vertex(double _x, double _y, double _z)
    : x{_x}, y{_y}, z{_z} {};

  explicit Vertex(Eigen::MatrixXd m)
    : x{m(0)}, y{m(1)}, z{m(2)} {};

  /** Take a vertex and transformation matrix; 
   *  return a transformed vertex.
   */
  static Vertex transform_vertex(Vertex v, Eigen::MatrixXd t) {
    using Eigen::MatrixXd;
    MatrixXd vertex_matrix(4, 1);
    vertex_matrix << v.x, v.y, v.z, 1;
    MatrixXd temp(4, 1);
    temp = t * vertex_matrix;
    double W = temp(3, 0);
    double x, y, z;
    x = temp(0, 0) / W;
    y = temp(1, 0) / W;
    z = temp(2, 0) / W;
    return Vertex(x, y, z);
  }

  /** Return a Vector3d of the vertex coordinates. **/
  Eigen::Vector3d matrix() const {
    return Eigen::Vector3d(x, y, z);
  }

  /** Check if all of the coords are between -1 and 1, inclusive.
   *  "inNDCCube" only applies if vertex is in NDC coords.
   */
  bool inNDCCube() {
    return x >= -1 && x <= 1 && y >= -1 && y <= 1 && z >= -1 && z <= 1;
  }
};

#endif  // VERTEX_H_
