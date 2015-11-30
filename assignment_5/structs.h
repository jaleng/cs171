#ifndef STRUCTS_H
#define STRUCTS_H

#include <vector>

struct Vec3d {
  double x, y, z;
};

struct Vertex {
  double x, y, z;
  explicit Vertex(double _x, double _y, double _z)
  : x{_x}, y{_y}, z{_z} {}

  /** Default constructor, initializes to (0,0,0) **/
  Vertex() : x{0}, y{0}, z{0} {}

  /** Divide (scaling) assignment operator **/
  const Vertex& operator/=(double s) {
    x /= s;
    y /= s;
    z /= s;
    return *this;
  }

  /** Normalize this vertex **/
  const Vertex& normalize() {
    auto magnitude = this->norm();
    this->operator/(magnitude);
    return *this;
  }

  /** Return copy of this vector normalized (do not normalize original) **/
  Vertex normalized() const {
    Vertex result = *this;
    result.normalize();
    return result;
  }

  /** Add assignment operator **/
  const Vertex& operator+=(const Vertex& rhs) {
    x += rhs.x;
    y += rhs.y;
    z += rhs.z;
    return *this;
  }

  /** Subtract assignment operator **/
  const Vertex& operator-=(const Vertex& rhs) {
    x -= rhs.x;
    y -= rhs.y;
    z -= rhs.z;
    return *this;
  }

  /** Return sum, do not alter originals **/
  Vertex operator+(const Vertex& rhs) const {
    Vertex v = *this;
    v += rhs;
    return v;
  }

  /** Return a - b, do not alter originals **/
  Vertex operator-(const Vertex& rhs) const {
    Vertex v = *this;
    v -= rhs;
    return v;
  }

  /** Return scaled, do not alter original **/
  Vertex operator/(const double s) const {
    return Vertex(x / s, y / s, z / s);
  }

  /** Return scaled, do not alter original **/
  Vertex operator*(const double s) const {
    return Vertex(x * s, y * s, z * s);
  }

  /** Return norm, aka magnitude, length **/
  double norm() {
    return sqrt(x * x + y * y + z * z);
  }
};

/** Face class used by halfedge structure **/
struct FaceforHE {
  int idx1, idx2, idx3;
  /** ctor, takes the vertex indices **/
  FaceforHE(int _idx1, int _idx2, int _idx3)
  : idx1{_idx1}, idx2{_idx2}, idx3{_idx3} {}
};


/** Struct to hold (pointers to) mesh data **/
struct Mesh_Data {
  std::vector<Vertex*> *vertices;
  std::vector<FaceforHE*> *faces;
};

#endif  // STRUCTS_H
