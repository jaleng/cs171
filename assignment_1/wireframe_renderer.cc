#include <memory>
#include <string>
#include <sstream>
#include <fstream>
#include <map>
#include <Eigen/Dense>
#include <iostream>

#include "Camera.h"
#include "ObjectData.h"
#include "ObjectCopyInfo.h"
#include "Scale.h"



/** Parse camera parameters and return a Camera unique_ptr. */
std::unique_ptr<Camera>
parse_camera(std::ifstream& file_stream) {
  using std::string;
  using std::stringstream;

  
  double px, py, pz;
  double ox, oy, oz, theta;
  double near, far, left, right, top, bottom;
  
  while (!file_stream.eof()) {
    string line;
    getline(file_stream, line);
    stringstream line_stream{line};
    string token;
    line_stream >> token;

    if (token == "camera:") {
      continue;
    } else if (token == "position") {
      line_stream >> px >> py >> pz;
    } else if (token == "orientation") {
      line_stream >> ox >> oy >> oz >> theta;
    } else if (token == "near") {
      line_stream >> near;
    } else if (token == "far") {
      line_stream >> far;
    } else if (token == "left") {
      line_stream >> left;
    } else if (token == "right") {
      line_stream >> right;
    } else if (token == "top") {
      line_stream >> top;
    } else if (token == "bottom") {
      line_stream >> bottom;
    } else {
      break;
    }
  }

  auto camera_up = std::unique_ptr<Camera> {
    new Camera{TranslationD{px, py, pz}, RotationD{ox, oy, oz, theta},
               Perspective{near, far, left, right, top, bottom}}};
  return std::move(camera_up);
}

// Parse object data
//  auto obj_to_data_up = parse_obj_to_data(scene_desc_file_stream);
std::unique_ptr<std::map<std::string, ObjectData>>
make_obj_to_data(const std::map<std::string, std::string>* obj_name_to_file_name_p) {
  using std::map;
  using std::string;
  using std::vector;
  using std::stringstream;
  using std::ifstream;

  std::unique_ptr<map<string, ObjectData>> obj_name_to_data_up {new map<string, ObjectData>};
  
  for (auto it = obj_name_to_file_name_p->begin(); it != obj_name_to_file_name_p->end(); ++it) {
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
    (*obj_name_to_data_up)[obj_name] = obj_data;
  }
  return std::move(obj_name_to_data_up);
}


// Parse object_copy attributes
//  auto obj_copy_info_vec_up = parse_obj_copy_info(scene_desc_file_stream);
std::unique_ptr<std::vector<ObjectCopyInfo>>
parse_obj_copy_info(std::ifstream& file_stream) {
  using std::string;
  using std::stringstream;
  using std::vector;
  using Eigen::MatrixXd;

  std::unique_ptr<vector<ObjectCopyInfo>> obj_copy_info_vec_up {new vector<ObjectCopyInfo>{}};

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
    (*obj_copy_info_vec_up).emplace_back(obj_name, transform);
  }
  return std::move(obj_copy_info_vec_up);
}



// Parse object attributes
// auto obj_name_to_filename_up = parse_obj_to_filename(scene_desc_file_stream);
std::unique_ptr<std::map<std::string, std::string>>
parse_obj_to_filename(std::ifstream& file_stream) {
  using std::map;
  using std::string;
  using std::stringstream;
  using mapss = map<string, string>;

  std::unique_ptr<mapss> obj_name_to_filename_up{new mapss};
  while (!file_stream.eof()) {
    string line;
    getline(file_stream, line);
    if (line.empty()) {
      break;
    }
    stringstream line_stream{line};
    string obj_name, file_name;
    line_stream >> obj_name >> file_name;
    (*obj_name_to_filename_up)[obj_name] = file_name;
  }
  
  return std::move(obj_name_to_filename_up);
  
}


// Store object_copy data (with transformed vertices)
// TODO(jg): make sure divide by w to get cartesian NDC.
// EX
//  auto obj_to_copy_data_vec = make_obj_to_copy_data_vec(obj_copy_info_vec_up.get(),
//                                                        obj_to_data_up.get());
// Store copy object data
std::unique_ptr<std::map<std::string, std::vector<ObjectData>>>
make_obj_to_copy_data_vec(std::vector<ObjectCopyInfo>* obj_copy_info_vec_p,
                          std::map<std::string, ObjectData>* obj_to_data_p) {
  using std::map;
  using std::string;
  using std::vector;

  using map_s_objdatavec = map<string, vector<ObjectData>>;

  std::unique_ptr<map_s_objdatavec> obj_to_vector_of_copy_data_up{new map_s_objdatavec};

  for (const auto& obj_copy_info : *obj_copy_info_vec_p) {
    auto obj_name = obj_copy_info.obj_id;
    vector<Vertex> vertices;
    MatrixXd transform = obj_copy_info.transform;
    for (const auto& untransformed_vertex : (*obj_to_data_p)[obj_name].vertices) {
      auto new_vertex = Vertex::transform_vertex(untransformed_vertex, transform);
      vertices.push_back(new_vertex);
    }
    auto it = (*obj_to_vector_of_copy_data_up).find(obj_name);
    if (it != (*obj_to_vector_of_copy_data_up).end()) {
      (*obj_to_vector_of_copy_data_up)[obj_name].emplace_back
        (std::move(vertices), (*obj_to_data_p)[obj_name].faces);
    } else {
      vector<ObjectData> od;
      od.emplace_back(std::move(vertices), (*obj_to_data_p)[obj_name].faces);
      (*obj_to_vector_of_copy_data_up)[obj_name] = std::move(od);
    }
  }
  return std::move(obj_to_vector_of_copy_data_up);
}


int main(int argc, char *argv[])
{
  /** Read the cli args. */
  // [scene_description_file.txt] [xres] [yres]
  std::ifstream scene_desc_file_stream{argv[1]};
  int xres = atoi(argv[2]);
  int yres = atoi(argv[3]);
  // DEBUG
  std::cout << xres << " " << yres << std::endl;
  // ENDEBUG

  /** Parse the scene description file into data structures. */
  // Parse camera attributes

  auto camera_p = parse_camera(scene_desc_file_stream);
  

  // Parse object attributes
  auto obj_name_to_filename_up = parse_obj_to_filename(scene_desc_file_stream);

  // Parse object_copy attributes
  auto obj_copy_info_vec_up = parse_obj_copy_info(scene_desc_file_stream);

  // Parse object data
  auto obj_to_data_up = make_obj_to_data(obj_name_to_filename_up.get());

  // Augment object_copy_info to transform points into NDC
  for (auto& obj_copy_info : (*obj_copy_info_vec_up)) {
    // Apply perspective projection transform
    auto tmp = obj_copy_info.transform;
    obj_copy_info.transform = camera_p->perspective_projection_matrix * tmp;
  }

  // Store object_copy data (with transformed vertices)
  auto obj_to_copy_data_vec = make_obj_to_copy_data_vec(obj_copy_info_vec_up.get(),
                                                        obj_to_data_up.get());

  // Mapping from NDC to pixel coordinates
  // multiply x by x_res / 2 then add x_res/2
  // multiplly y by y_res / 2 then add y_res/2

  // Use Bresenham's line algorithm

  // Output pixel grid to stdout as a .ppm image file
  return 0;
}
