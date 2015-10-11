#include <vector>
#include <string>
#include <fstream>
#include <string>
#include <sstream>
#include <iostream>
#include "Face.h"
#include "Vertex.h"
#include "ObjectData.h"

void print_object_data(const ObjectData& object_data) {
  for (const auto& vertex : object_data.vertices) {
    std::cout << "v " << vertex.x << " " << vertex.y << " " << vertex.z;
    std::cout << std::endl;
  }
  for (const auto& face : object_data.faces) {
    std::cout << "f " << face.v1_idx << " " << face.v2_idx << " " << face.v3_idx;
    std::cout << std::endl;
  }

}

/** Parse the passed in files, store the object information, and print it. */
int main(int argc, char *argv[])
{
  using std::vector;
  using std::ifstream;
  using std::string;
  using std::stringstream;
  
  vector<ObjectData> file_obj_data_vec;
  for (int i = 1; i < argc; ++i) {
    ifstream fileStream{argv[i]};
    vector<Vertex> vertices;
    vector<Face> faces;

    while (!fileStream.eof()) {
      string line;
      getline(fileStream, line);
      stringstream line_stream{line};
      char shape_token = line_stream.get();
      if (shape_token == 'v') {
        float x, y, z;
        line_stream >> x >> y >> z;
        vertices.emplace_back(x, y, z);
      } else if (shape_token == 'f') {
        int v1, v2, v3;
        line_stream >> v1 >> v2 >> v3;
        faces.emplace_back(v1, v2, v3);
      }
    }
    file_obj_data_vec.emplace_back(std::move(vertices), std::move(faces));
  }

  // print out the data
  int file_idx = 1;
  for (const auto& obj_data : file_obj_data_vec) {
    std::cout << string{argv[file_idx]} << ":" << std::endl;
    print_object_data(obj_data);
    std::cout << std::endl;
    ++file_idx;
  }

  return 0;
}
