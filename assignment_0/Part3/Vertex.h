#ifndef VERTEX_H_
#define VERTEX_H_

#include <Eigen/Dense>

/** Hold 3 float coordinates for x,y,z space */
class Vertex {
public:
  float x;
  float y;
  float z;

  explicit Vertex(float _x, float _y, float _z)
    : x{_x}, y{_y}, z{_z} {};

  /** Take a vertex and transformation matrix; 
   *  return a transformed vertex.
   */
  static Vertex transform_vertex(Vertex v, Eigen::MatrixXd t) {
    using Eigen::MatrixXd;
    MatrixXd vertex_matrix(4,1);
    vertex_matrix << v.x, v.y, v.z, 1;
    MatrixXd temp(4,1);
    temp = t * vertex_matrix;
    float W = temp(3,0);
    float x, y, z;
    x = temp(0,0) / W;
    y = temp(1,0) / W;
    z = temp(2,0) / W;
    return Vertex(x, y, z);
  }
};

#endif // VERTEX_H_
