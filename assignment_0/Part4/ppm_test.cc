#include <Eigen/Dense>
#include <iostream>
#include <string>

/** Make a ppm image of the given dimensions
 *  with a circle in the center
 */
int main(int argc, char *argv[])
{
  using Eigen::MatrixXd;
  using std::cout;
  using std::endl;

  int xres = atoi(argv[1]);
  int yres = atoi(argv[2]);
  int radius = xres < yres ? xres/4 : yres/4;
  
  MatrixXd pixels(xres,yres);
  pixels = MatrixXd::Zero(xres,yres);
  for (int i = 0; i < xres; ++i) {
    for (int j = 0; j < yres; ++j) {
      float i_f = static_cast<float>(i);
      float j_f = static_cast<float>(j);
      float x = i_f - static_cast<float>(xres)/2;
      float y = j_f - static_cast<float>(yres)/2;
      if (x * x + y * y < radius * radius) {
        pixels(i,j) = 1;
      }
    }
  }

  cout << "P3" << endl
       << xres << " " << yres << endl
       << "255" << endl;

  for (int j = 0; j < yres; ++j) {
    for (int i = 0; i < xres; ++i) {
      if (pixels(i,j) == 0) {
        cout << "255 255 255" << endl;
      } else {
        cout << "0 0 0" << endl;
      }
    }
  }

  return 0;
}
