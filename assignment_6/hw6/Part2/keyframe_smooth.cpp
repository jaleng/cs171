#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>

#include <Eigen/Dense>
#include "animation.h"

using std::vector;
using std::string;
using std::stringstream;
using std::ifstream;

/** Simple class to hold vertex components **/
class Vertex {
 public:
  double pos[3];

  Vertex(double x, double y, double z) : pos{x, y, z} {}

  double& operator[](int i) {
    return pos[i];
  }
};

/** Hold vertex info for each frame in a shape animation **/
class ShapeAnimation {
 public:
  size_t num_frames;
  vector<int> keyframes;
  vector<vector<Vertex>> frame_vertices;

  // ctor
  explicit ShapeAnimation(size_t _num_frames)
  : num_frames{_num_frames}, frame_vertices{num_frames} {}

  /** Interpolate vertices from keyframes **/
  void interpolateVertices() {
    using Eigen::Matrix;
    using Eigen::MatrixXd;
    for (auto i = 0; i < keyframes.size() - 1; ++i) {
      initialize_catmull_rom_B();

      // get keyframe indices, at the beginning, treat 0th as i and i-1
      // at the end, treat last keyframe index as i+1 and i+2
      int im1 = (i == 0) ? 0 : i - 1;
      int ip1 = i + 1;
      int ip2 = (i == keyframes.size() - 2) ? keyframes.size() - 1 : i + 2;
      // Change keyframe indices to frame numbers
      int fi = keyframes[i];
      int fim1 = keyframes[im1];
      int fip1 = keyframes[ip1];
      int fip2 = keyframes[ip2];

      int n_vertices = frame_vertices[fi].size();
      // Build p matrix
      MatrixXd p(4, 3 * n_vertices);
      for (int v = 0; v < n_vertices; ++v) {
        p(0, 3*v) = frame_vertices[fim1][v][0];
        p(1, 3*v) = frame_vertices[fi  ][v][0];
        p(2, 3*v) = frame_vertices[fip1][v][0];
        p(3, 3*v) = frame_vertices[fip2][v][0];

        p(0, 3*v + 1) = frame_vertices[fim1][v][1];
        p(1, 3*v + 1) = frame_vertices[fi  ][v][1];
        p(2, 3*v + 1) = frame_vertices[fip1][v][1];
        p(3, 3*v + 1) = frame_vertices[fip2][v][1];

        p(0, 3*v + 2) = frame_vertices[fim1][v][2];
        p(1, 3*v + 2) = frame_vertices[fi  ][v][2];
        p(2, 3*v + 2) = frame_vertices[fip1][v][2];
        p(3, 3*v + 2) = frame_vertices[fip2][v][2];
      }

      MatrixXd Bp;
      Bp = catmull_rom_B * p;

      double du = 1.0 / (fip1 - fi);
      double u = du;
      for (int inbetween = fi + 1; inbetween < fip1; ++inbetween) {
        Matrix<double, 1, 4> u_m;
        u_m << 1.0, u, u*u, u*u*u;
        auto interpolated = u_m*Bp;
        for (int v = 0; v < n_vertices; ++v) {
          double x = interpolated(0, 3*v);
          double y = interpolated(0, 3*v + 1);
          double z = interpolated(0, 3*v + 2);
          frame_vertices[inbetween].emplace_back(x, y, z);
        }
        u += du;
      }
    }
  }
};

/** Check if interpolation is correct.
 * Very specific to the bunny test case.
 * Assumes obj files are in the same directory.
 */
bool checkInterpolated(ShapeAnimation& sa) {
  using std::ifstream;
  using std::string;
  using std::vector;

  vector<string> filenames {"",
                            "bunny01.obj",
                            "bunny02.obj",
                            "bunny03.obj",
                            "bunny04.obj",
                            "",
                            "bunny06.obj",
                            "bunny07.obj",
                            "bunny08.obj",
                            "bunny09.obj",
                            "",
                            "bunny11.obj",
                            "bunny12.obj",
                            "bunny13.obj",
                            "bunny14.obj",
                            "",
                            "bunny16.obj",
                            "bunny17.obj",
                            "bunny18.obj",
                            "bunny19.obj"};

  for (int i = 0; i < filenames.size(); ++i) {
    if (filenames[i].empty()) {
      continue;
    }

    ifstream fs{filenames[i]};
    int v = 0;
    while (!fs.eof()) {
      string line;
      getline(fs, line);
      if (line.empty()) {
        break;
      }
      stringstream linestream{line};
      string token;
      linestream >> token;
      if (token == "v") {
        double expectedx, expectedy, expectedz;
        linestream >> expectedx >> expectedy >> expectedz;
        double x = sa.frame_vertices[i][v][0];
        double y = sa.frame_vertices[i][v][1];
        double z = sa.frame_vertices[i][v][2];
        if (abs(x - expectedx) > .001
            || abs(y - expectedy) > .001
            || abs(z - expectedz) > .001) {
          return false;
        }
        ++v;
        // dfasdfdsafdsf
      } else {
        break;
      }
    }
  }
  return true;
}

/** Read Vertices from obj file **/
vector<Vertex> readVertices(ifstream& fs) {
  vector<Vertex> res;
  string line;
  while (!fs.eof()) {
    getline(fs, line);
    if (line.empty()) {
      break;
    }
    stringstream ss{line};
    string token;
    double x, y, z;
    ss >> token >> x >> y >> z;
    if (token == "v") {
      res.emplace_back(x, y, z);
    } else {
      break;
    }
  }
  return res;
}

int main(int argc, char *argv[]) {
  auto shape_animation = ShapeAnimation{21};
  shape_animation.keyframes = vector<int>{0, 5, 10, 15, 20};

  ifstream f00 {"bunny00.obj"};
  shape_animation.frame_vertices[0] = readVertices(f00);
  ifstream f05 {"bunny05.obj"};
  shape_animation.frame_vertices[5] = readVertices(f05);
  ifstream f10 {"bunny10.obj"};
  shape_animation.frame_vertices[10] = readVertices(f10);
  ifstream f15 {"bunny15.obj"};
  shape_animation.frame_vertices[15] = readVertices(f15);
  ifstream f20 {"bunny20.obj"};
  shape_animation.frame_vertices[20] = readVertices(f20);

  shape_animation.interpolateVertices();

  if (checkInterpolated(shape_animation)) {
    std::cout << "CORRECT: interpolated frames match\n";
  } else {
    std::cout << "INCORRECT: interpolated frames do not match\n";
  }

  return 0;
}

