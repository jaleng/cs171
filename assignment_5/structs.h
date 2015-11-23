#ifndef STRUCTS_H
#define STRUCTS_H

#include <vector>

struct Vec3f
{
	float x, y, z;
};

struct Vertex
{
  float x, y, z;
  explicit Vertex(float _x, float _y, float _z)
  : x{_x}, y{_y}, z{_z} {}

  /** Default constructor, initializes to (0,0,0) **/
  Vertex() : x{0}, y{0}, z{0} {}

  const Vertex& operator/=(float s) {
    x /= s;
    y /= s;
    z /= s;
    return *this;
  }

  const Vertex& normalize() {
    auto magnitude = this->norm();
    this->operator/(magnitude);
    return *this;
  }

  Vertex normalized() const {
    Vertex result = *this;
    result.normalize();
    return result;
  }

  const Vertex& operator+=(const Vertex& rhs) {
    x += rhs.x;
    y += rhs.y;
    z += rhs.z;
    return *this;
  }

  const Vertex& operator-=(const Vertex& rhs) {
    x -= rhs.x;
    y -= rhs.y;
    z -= rhs.z;
    return *this;
  }

  Vertex operator+(const Vertex& rhs) const {
    Vertex v = *this;
    v += rhs;
    return v;
  }

  Vertex operator-(const Vertex& rhs) const {
    Vertex v = *this;
    v -= rhs;
    return v;
  }

  Vertex operator/(const float s) const {
    return Vertex(x / s, y / s, z / s);
  }

  Vertex operator*(const float s) const {
    return Vertex(x * s, y * s, z * s);
  }

  float norm() {
    return sqrt(x * x + y * y + z * z);
  }
};

struct FaceforHE
{
    int idx1, idx2, idx3;
  FaceforHE(int _idx1, int _idx2, int _idx3)
  : idx1{_idx1}, idx2{_idx2}, idx3{_idx3} {}
};

struct Mesh_Data
{
    std::vector<Vertex*> *vertices;
    std::vector<FaceforHE*> *faces;
};

#endif
