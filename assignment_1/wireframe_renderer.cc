


/** Parse camera parameters and return a Camera unique_ptr. */
std::unique_ptr<Camera>
parse_camera(std::ifstream file_stream) {
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



int main(int argc, char *argv[])
{
  /** Read the cli args. */
  // [scene_description_file.txt] [xres] [yres]
  ifstream scene_desc_file_stream{argv[1]};
  int xres = atoi(argv[2]);
  int yres = atoi(argv[3]);

  /** Parse the scene description file into data structures. */
  // Parse camera attributes
  // TODO(jg): When camera is constructed, make it hold the
  //           camera transform, inv cam transform, and perspective projection matrix.

  auto camera_p = parse_camera(scene_desc_file_stream);
  

  // Parse object attributes
  auto obj_name_to_filename_up = parse_obj_to_filename(scene_desc_file_stream);

  // Parse object_copy attributes
  auto obj_copy_info_vec_up = parse_obj_copy_info(scene_desc_file_stream);

  // Parse object data
  auto obj_to_data_up = parse_obj_to_data(scene_desc_file_stream);

  // Augment object_copy_info to transform points into NDC
  for (auto& obj_copy_info : obj_copy_info_vec) {
    // Apply perspective projection transform
    auto tmp = obj_copy_info.transform;
    obj_copy_info.transform = camera_p->perspective_projection_matrix * tmp;
  }

  // Store object_copy data (with transformed vertices)
  // TODO(jg): make sure divide by w to get cartesian NDC.
  auto obj_to_copy_data_vec = make_obj_to_copy_data_vec(obj_copy_info_vec_up.get(),
                                                        obj_to_data_up.get());

  // Mapping from NDC to pixel coordinates
  // multiply x by x_res / 2 then add x_res/2
  // multiplly y by y_res / 2 then add y_res/2

  // Use Bresenham's line algorithm

  // Output pixel grid to stdout as a .ppm image file
  return 0;
}
