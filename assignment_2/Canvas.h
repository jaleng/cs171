#ifndef CANVAS_H
#define CANVAS_H

#include <Eigen/Dense>

#include "Vertex.h"
#include "ObjectData.h"
#include "Point.h"

/** Class to hold a pixel grid, with functions to fill points and draw lines */
class Canvas {
 public:
  // Data members
  int width;
  int height;
  Eigen::MatrixXd pixels;
  
  // Constructors

  /** Construct canvas with given width and height */
  explicit Canvas(int _width, int _height)
    : width{_width}, height{_height}, pixels{_width, _height} {
    pixels = Eigen::MatrixXd::Zero(3 * width, height);
  }

  // Mutators

  /** "Fill" in a pixel of the pixel grid */
  void fill(int x, int y, Color color) {
    pixels(3 * x, y) = color.red;
    pixels(3 * x + 1, y) = color.green;
    pixels(3 * x + 2, y) = color.blue;
  }

  // Utility
  /** Get point to represent NDC vertex in canvas-space. */
  Point getPointFromNDCVertex(Vertex& v) {
    int x = round((v.x + 1) * (width / 2 - 1));
    int y = round((-v.y + 1) * (height / 2 - 1));
    return Point{x, y};
  }
};

#endif // CANVAS_H
