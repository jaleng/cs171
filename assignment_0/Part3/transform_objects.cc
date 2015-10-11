#include <vector>
#include <string>
#include <map>
#include <fstream>
#include <string>
#include <sstream>
#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include "ObjectCopyInfo.h"
#include "ObjectData.h"
#include "Translation.h"
#include "Rotation.h"
#include "Scale.h"


// read obj names and filenames, store in obj_name_to_file_name
void fill_obj_id_to_filename_map(
       std::map<std::string,std::string>& obj_name_to_file_name,
       std::ifstream& file_stream) {
  using std::string;
  using std::stringstream;
  while (!file_stream.eof()) {
    string line;
    getline(file_stream, line);
    if (line.empty()) {
      break;
    }
    stringstream line_stream{line};
    string obj_name, file_name;
    line_stream >> obj_name >> file_name;
    obj_name_to_file_name[obj_name] = file_name;
  }
}


void fill_obj_copy_info_vec(std::vector<ObjectCopyInfo>& obj_copy_info_vec,
                            std::ifstream& file_stream) {
  using std::string;
  using std::stringstream;
  using std::vector;
  using Eigen::MatrixXd;

  while (!file_stream.eof()) {
    // read 1 obj and its transformations
    string line;
    getline(file_stream, line);
    if (line.empty()) {
      break;
    }
    stringstream line_stream{line};
    string obj_name;
    line_stream >> obj_name;

    MatrixXd transform(4,4);
    MatrixXd next_transform(4, 4);
    MatrixXd temp(4,4);
    transform = MatrixXd::Identity(4, 4);
    
    for (getline(file_stream, line); !line.empty(); getline(file_stream, line)) {
      stringstream transform_line_stream(line);
      char shape_token = transform_line_stream.get();

      if (shape_token == 't') {
        double tx, ty, tz;
        transform_line_stream >> tx >> ty >> tz;
        TranslationD::translation_vals_to_matrix(next_transform, tx, ty, tz);
      
      } else if (shape_token == 'r') {
        double rx, ry, rz, theta;
        transform_line_stream >> rx >> ry >> rz >> theta;
        RotationD::rotation_vals_to_matrix(next_transform, rx, ry, rz, theta);

      } else if (shape_token == 's') {
        double sx, sy, sz;
        transform_line_stream >> sx >> sy >> sz;
        ScaleD::scale_vals_to_matrix(next_transform, sx, sy, sz);
      
      } else {
        continue;
      }

      temp = next_transform * transform;
      transform = temp;
      
    }

    // Store (name,transform) into a vector
    obj_copy_info_vec.emplace_back(obj_name, transform);
  }
  
}
  // Get the ObjectData (vertices, faces) for each file
  // map holding object name -> ObjectData(vertices and faces)
void fill_obj_name_to_data(
       std::map<std::string, ObjectData>& obj_name_to_data,
       std::map<std::string,std::string>& obj_name_to_file_name,
       std::ifstream& file_stream
       ) {
  using std::map;
  using std::string;
  using std::vector;
  using std::stringstream;
  using std::ifstream;

  for (auto it = obj_name_to_file_name.begin(); it != obj_name_to_file_name.end(); ++it) {
    string obj_name = it->first;
    string file_name = it->second;
    ifstream obj_fileStream{file_name};
    vector<Vertex> vertices;
    vector<Face> faces;

    while (!obj_fileStream.eof()) {
      string line;
      getline(obj_fileStream, line);
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
    ObjectData obj_data{std::move(vertices), std::move(faces)};
    obj_name_to_data[obj_name] = obj_data;
  }
}

// Store copy object data
void fill_obj_name_to_vector_of_copy_data(
       std::map<std::string, std::vector<ObjectData>>& obj_name_to_vector_of_copy_data,
       std::vector<ObjectCopyInfo>& obj_copy_info_vec,
       std::map<std::string, ObjectData>& obj_name_to_data
       ) {
  using std::map;
  using std::string;
  using std::vector;

  for (const auto& obj_copy_info : obj_copy_info_vec) {
    auto obj_name = obj_copy_info.obj_id;
    vector<Vertex> vertices;
    MatrixXd transform = obj_copy_info.transform;
    for (const auto& untransformed_vertex : obj_name_to_data[obj_name].vertices) {
      auto new_vertex = Vertex::transform_vertex(untransformed_vertex, transform);
      vertices.push_back(new_vertex);
    }
    auto it = obj_name_to_vector_of_copy_data.find(obj_name);
    if (it != obj_name_to_vector_of_copy_data.end()) {
      obj_name_to_vector_of_copy_data[obj_name].emplace_back
        (std::move(vertices), vector<Face>());
    } else {
      vector<ObjectData> od;
      od.emplace_back(std::move(vertices), vector<Face>());
      obj_name_to_vector_of_copy_data[obj_name] = std::move(od);
    }
  }
}

/*
 * Read in a formatted text file with the following format
 * --------------------
 * object1_identifier object1_filename.obj
 * object2_identifier object2_filename.obj
 * ...
 * [blank line]
 * object_identifier
 * [t tx ty tz | s sx sy sz | r rx ry rz angle]
 * ...
 * [blank line]
 * ...
 * --------------------
 * And print out the vertices of the object copies with the given transforms
 */

int main(int argc, char *argv[])
{
  using std::vector;
  using std::ifstream;
  using std::string;
  using std::stringstream;
  using std::cout;
  using std::map;
  using Eigen::MatrixXd;

  // Get file name from command line argument
  ifstream fileStream{argv[1]};

  // map name of object (identifier) -> file name
  std::map<std::string,std::string> obj_name_to_file_name;
  fill_obj_id_to_filename_map(obj_name_to_file_name, fileStream);


  // read obj copies
  // store ObjectCopyInfos for the object copies read from file
  // ObjectCopyInfo has obj_name and transformation
  vector<ObjectCopyInfo> obj_copy_info_vec;
  fill_obj_copy_info_vec(obj_copy_info_vec, fileStream);

  // Get the ObjectData (vertices, faces) for each file
  // map holding object name -> ObjectData(vertices and faces)
  map<string, ObjectData> obj_name_to_data;
  fill_obj_name_to_data(obj_name_to_data, obj_name_to_file_name, fileStream);

  // Store copy object data
  map<string, vector<ObjectData>> obj_name_to_vector_of_copy_data;
  fill_obj_name_to_vector_of_copy_data(
    obj_name_to_vector_of_copy_data,
    obj_copy_info_vec,
    obj_name_to_data);

  // output transformed vertices for each output
  for (auto it = obj_name_to_vector_of_copy_data.begin();
       it != obj_name_to_vector_of_copy_data.end();
       ++it) {
    auto obj_name = it->first;
    auto copy_vec = it->second;
    for (uint i = 0; i < copy_vec.size(); ++i) {
      cout << obj_name << "_copy" << i << std::endl;
      for (auto vertex : copy_vec[i].vertices) {
        cout << vertex.x << " " << vertex.y << " " << vertex.z << std::endl;
      }
      cout << std::endl;
    }
  }
}
