#include "SceneParser.h"

#include <memory>
#include "Triple.h"

/** Parse a scene given a scene description file stream. **/
std::unique_ptr<Scene>
SceneParser::parse_scene(std::ifstream& scene_desc_file_stream) {
  // Parse the camera
  auto camera_up = parse_camera(scene_desc_file_stream);
  // Parse lights
  auto light_vec_up = parse_lights(scene_desc_file_stream);
  // Parse Objects (obj_name, filename)
  auto objid_to_filename_up = parse_obj_to_filename(scene_desc_file_stream);
  // Parse Object-copies
  auto obj_copy_info_vec_up = parse_obj_copy_info(scene_desc_file_stream);
  // Read .obj files
  auto objid_to_data_up = make_obj_to_data(objid_to_filename_up.get());

  // Make vector of Objects
  auto obj_vec_up = make_obj_vec(obj_copy_info_vec_up.get(),
                                 objid_to_data_up.get());
  // Return a scene object
  return std::make_unique<Scene>(std::move(camera_up),
                                 std::move(light_vec_up),
                                 std::move(objid_to_data_up),
                                 std::move(obj_vec_up));
}

/** Parse camera parameters and return a Camera unique_ptr. */
std::unique_ptr<Camera>
SceneParser::parse_camera(std::ifstream& file_stream) {
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

  return std::make_unique<Camera>(px, py, pz,
                                  ox, oy, oz, theta,
                                  near, far, left, right, top, bottom);
}

/** Parse lights **/
std::unique_ptr<std::vector<PointLight>>
SceneParser::parse_lights(std::ifstream& file_stream) {
  using std::string;
  using std::stringstream;
  using std::vector;

  auto light_vec_up = std::make_unique<vector<PointLight>>();
  while (!file_stream.eof()) {
    string line;
    getline(file_stream, line);
    if (line.empty()) {
      // Empty line, no more lights.
      break;
    }
    stringstream line_stream{line};
    string token;
    line_stream >> token;
    double x, y, z;
    double r, g, b;
    double k;
    string spacer;

    if (token == "light") {
      // Read position coordinates.
      line_stream >> x >> y >> z;

      // Read comma between position and rgb values.
      line_stream >> spacer;
      assert(spacer == ",");

      // Read rgb values
      line_stream >> r >> g >> b;

      // Read comma between rgb values and attenuation value.
      line_stream >> spacer;
      assert(spacer == ",");

      // Read attenuation value.
      line_stream >> k;

      // Push new light to vector.
      light_vec_up->emplace_back(x, y, z, r, g, b, k);
    } else {
      assert(false);  // Parsing PointLights error.
      break;
    }
  }
  return std::move(light_vec_up);
}

/** Parse object data. */
std::unique_ptr<std::map<std::string, ObjectData>>
SceneParser::make_obj_to_data(const std::map<std::string,
                              std::string>* obj_name_to_file_name_p) {
  using std::map;
  using std::string;
  using std::vector;
  using std::stringstream;
  using std::ifstream;

  std::unique_ptr<map<string, ObjectData>>
    obj_name_to_data_up {new map<string, ObjectData>};

  for (auto it = obj_name_to_file_name_p->begin();
       it != obj_name_to_file_name_p->end();
       ++it) {
    string obj_name = it->first;
    string file_name = it->second;
    ifstream obj_fileStream{file_name};
    vector<Triple> vertices;
    vector<Triple> normals;
    vector<Face> faces;

    while (!obj_fileStream.eof()) {
      string line;
      getline(obj_fileStream, line);
      stringstream line_stream{line};
      string token;
      line_stream >> token;
      if (token == "v") {
        double x, y, z;
        line_stream >> x >> y >> z;
        vertices.emplace_back(x, y, z);
      } else if (token == "vn") {
        // No more normals being read in, obsolete code
        assert(false);  // Parsing error
        double x, y, z;
        line_stream >> x >> y >> z;
        normals.emplace_back(x, y, z);
      } else if (token == "f") {
        int idx1, idx2, idx3;
        line_stream >> idx1 >> idx2 >> idx3;
        // Setting normal indices to 0 to indicate no norms given
        // and still use previous code
        faces.emplace_back(idx1, idx2, idx3, 0, 0, 0);
      }
    }
    ObjectData obj_data{std::move(vertices), std::move(normals), std::move(faces)};
    (*obj_name_to_data_up)[obj_name] = obj_data;
  }
  return std::move(obj_name_to_data_up);
}

/** Parse object_copy attributes. */
std::unique_ptr<std::vector<ObjectCopyInfo>>
SceneParser::parse_obj_copy_info(std::ifstream& file_stream) {
  using std::string;
  using std::stringstream;
  using std::vector;

  auto obj_copy_info_vec_up = std::make_unique<vector<ObjectCopyInfo>>();

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

    double amb_r = 0, amb_g = 0, amb_b = 0;
    double dif_r = 0, dif_g = 0, dif_b = 0;
    double spc_r = 0, spc_g = 0, spc_b = 0;
    double shininess = 0;

    std::vector<Transform> transforms;

    for (getline(file_stream, line); !line.empty(); getline(file_stream, line)) {
      stringstream transform_line_stream(line);
      string token;
      transform_line_stream >> token;

      if (token == "ambient") {
        transform_line_stream >> amb_r >> amb_g >> amb_b;
        continue;
      } else if (token == "diffuse") {
        transform_line_stream >> dif_r >> dif_g >> dif_b;
        continue;
      } else if (token == "specular") {
        transform_line_stream >> spc_r >> spc_g >> spc_b;
        continue;
      } else if (token == "shininess") {
        transform_line_stream >> shininess;
        continue;
      } else if (token == "t") {
        double tx, ty, tz;
        transform_line_stream >> tx >> ty >> tz;
        transforms.emplace_back(TransformType::TRANSLATION, tx, ty, tz);
      } else if (token == "r") {
        double rx, ry, rz, theta;
        transform_line_stream >> rx >> ry >> rz >> theta;
        transforms.emplace_back(TransformType::ROTATION, rx, ry, rz, theta);
      } else if (token == "s") {
        double sx, sy, sz;
        transform_line_stream >> sx >> sy >> sz;
        transforms.emplace_back(TransformType::SCALING, sx, sy, sz);
      } else {
        assert(false);  // Error parsing object copy.
        continue;
      }
    }

    // Store (name,transform) into a vector
    (*obj_copy_info_vec_up).emplace_back(obj_name,
                                         amb_r, amb_g, amb_b,
                                         dif_r, dif_g, dif_b,
                                         spc_r, spc_g, spc_b,
                                         shininess,
                                         transforms);
  }
  return std::move(obj_copy_info_vec_up);
}

/** Parse object id and filename associations. */
std::unique_ptr<std::map<std::string, std::string>>
SceneParser::parse_obj_to_filename(std::ifstream& file_stream) {
  using std::map;
  using std::string;
  using std::stringstream;
  using mapss = map<string, string>;

  // Read until "objects:" line
  while (!file_stream.eof()) {
    string line;
    getline(file_stream, line);
    if (line == "objects:") {
      break;
    }
  }

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


/** Make a vector of Objects using object data and copy info. **/
std::unique_ptr<std::vector<Object>>
SceneParser::make_obj_vec(std::vector<ObjectCopyInfo> *objcpy_info_vec_p,
                  std::map<std::string, ObjectData> *objid_to_data_p) {
  using std::vector;
  using std::string;
  using std::unique_ptr;

  auto obj_vec_up = std::make_unique<vector<Object>>();
  for (auto& objcpy_info : *objcpy_info_vec_p) {
    Object obj{};
    obj.object_id = objcpy_info.obj_id;
    for (int i = 0; i < 3; ++i) {
      obj.ambient_reflect[i] = objcpy_info.ambient_reflectance[i];
      obj.diffuse_reflect[i] = objcpy_info.diffuse_reflectance[i];
      obj.specular_reflect[i] = objcpy_info.specular_reflectance[i];
    }
    obj.shininess = objcpy_info.shininess;

    auto obj_data = (*objid_to_data_p)[obj.object_id];

    for (const auto& face : obj_data.faces) {
      obj.vertex_buffer.push_back(obj_data.vertices[face.v1_idx - 1]);
      obj.vertex_buffer.push_back(obj_data.vertices[face.v2_idx - 1]);
      obj.vertex_buffer.push_back(obj_data.vertices[face.v3_idx - 1]);
    }

    obj.transforms = objcpy_info.transforms;

    obj_vec_up->push_back(std::move(obj));
  }
  return std::move(obj_vec_up);
}
