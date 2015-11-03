// shaded_renderer.cc
#include <fstream>
#include <cassert>
#include <float.h>
#include <iostream>

#include "SceneParser.h"
#include "Scene.h"
#include "Lighting.h"

/** Take canvas and generate a ppm file, output to stdout. */
void ppm_to_stdout(Canvas& c) {
  using std::cout;
  using std::endl;
  
  cout << "P3" << endl
       << c.width << " " << c.height << endl
       << "255" << endl;

  for (int j = 0; j < c.height; ++j) {
    for (int i = 0; i < c.width; ++i) {
      cout << round(255 * c.pixels(3 * i, j)) << " "
           << round(255 * c.pixels(3 * i + 1, j)) << " "
           << round(255 * c.pixels(3 * i + 2, j)) << endl;
    }
  }
}

int main(int argc, char *argv[]) {
  using namespace Lighting;

  // Get cli args:
  // ./shaded_renderer [scene_description_file.txt] [xres] [yres] [mode]
  std::ifstream scene_desc_file_stream{argv[1]};
  int xres = atoi(argv[2]);
  int yres = atoi(argv[3]);

  int mode_int = atoi(argv[4]);
  assert(mode_int == 0 || mode_int == 1);
  // enum class ShadingMode {GOURAUD, PHONG};
  // ShadingMode shading_mode = mode_int == 0 ?
  //   ShadingMode::GOURAUD : ShadingMode::PHONG;

  // Parse scene description file
  auto scene = SceneParser::parse_scene(scene_desc_file_stream);
  auto canvas = Canvas{xres, yres};

  Eigen::MatrixXd buffer(xres, yres);
  buffer.fill(DBL_MAX);
  for (const auto& obj_copy : *((*scene).object_copies_up)) {
    for (const auto& face : obj_copy.faces) {
      rasterColoredTriangle(face,
                            obj_copy.vertices,
                            obj_copy.normals,
                            obj_copy.material,
                            *((*scene).camera_up),
                            *((*scene).lights_up),
                            canvas,
                            buffer,
                            mode_int);
    }
  }

  // DEBUG CANVAS
  // auto debug_canvas = Canvas{800, 800};
  // for (int i = 200; i <= 400; ++i) {
  //   for (int j = 200; j <= 400; ++j) {
  //     debug_canvas.fill(i, j, Color{1, 0, 0});
  //   }
  // }

  // output to ppm
  ppm_to_stdout(canvas);

  return 0;
}
