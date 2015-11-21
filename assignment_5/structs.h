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
