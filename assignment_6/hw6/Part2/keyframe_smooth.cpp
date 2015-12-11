#include <vector>
#include <string>
#include <sstream>
#include <fstream>

using std::vector;
using std::string;
using std::stringstream;
using std::ifstream;

class Vertex {
 public:
  double pos[3];

  Vertex(double x, double y, double z) : pos{x, y, z} {}

  double& operator[](int i) {
    return pos[i];
  }
};


class ShapeAnimation {
 public:
  size_t num_frames;
  vector<int> keyframes;
  vector<vector<Vertex>> frame_vertices;

  // ctor
  explicit ShapeAnimation(size_t _num_frames)
  : num_frames{_num_frames}, frame_vertices{num_frames} {}
};

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

  
  return 0;
}

