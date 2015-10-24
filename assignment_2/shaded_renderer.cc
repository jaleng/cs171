// shaded_renderer.cc

int main(int argc, char *argv[])
{
  // Get cli args: ./shaded_renderer [scene_description_file.txt] [xres] [yres] [mode]
  std::ifstream scene_desc_file_stream{argv[1]};
  int xres = atoi(argv[2]);
  int yres = atoi(argv[1]);

  int mode_int = atoi(argv[3]);
  assert(mode_int == 0 || mode_int == 1);
  enum class ShadingMode {GOURAUD, PHONG};
  ShadingMode shading_mode = mode_int == 0 ? ShadingMode::GOURAUD : ShadingMode::PHONG;
  
  // Parse scene description file
  return 0;
}
