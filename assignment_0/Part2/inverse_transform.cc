#include <vector>
#include <string>
#include <fstream>
#include <string>
#include <sstream>
#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include "Translation.h"
#include "Rotation.h"
#include "Scale.h"

int main(int argc, char *argv[])
{
  using std::vector;
  using std::ifstream;
  using std::string;
  using std::stringstream;
  
  ifstream fileStream{argv[1]};

  MatrixXd transform(4,4);
  MatrixXd next_transform(4, 4);
  MatrixXd temp(4,4);
  transform = MatrixXd::Identity(4, 4);

  while (!fileStream.eof()) {
    string line;
    getline(fileStream, line);
    stringstream line_stream{line};
    char shape_token = line_stream.get();

    if (shape_token == 't') {
      double tx, ty, tz;
      line_stream >> tx >> ty >> tz;
      TranslationD::translation_vals_to_matrix(next_transform, tx, ty, tz);
      
    } else if (shape_token == 'r') {
      double rx, ry, rz, theta;
      line_stream >> rx >> ry >> rz >> theta;
      RotationD::rotation_vals_to_matrix(next_transform, rx, ry, rz, theta);

    } else if (shape_token == 's') {
      double sx, sy, sz;
      line_stream >> sx >> sy >> sz;
      ScaleD::scale_vals_to_matrix(next_transform, sx, sy, sz);
      
    } else {
      continue;
    }
    /*// DEBUG
    std::cout << next_transform << std::endl << std::endl;
    //*/// ENDEBUG
    temp = next_transform * transform;
    transform = temp;
  }

  std::cout << transform.inverse();
  
  return 0;
}
