#ifndef CANVAS_H
#define CANVAS_H

#include <Eigen/Dense>

#include "Vertex.h"
#include "ObjectData.h"
#include "Point.h"

class Canvas {
public:
  // Data members
  int width;
  int height;
  Eigen::MatrixXd pixels;
  
  // Constructors
  explicit Canvas(int _width, int _height)
    : width{_width}, height{_height}, pixels{_width, _height} {
    pixels = Eigen::Zeros(width, height);
  }

  // Mutators
  void drawLine(Point p1, Point p2) {
    // TODO(jg)
  }

  void drawLine(Vertex& v1, Vertex& v2) {
    // TODO(jg)
    
  }

  void drawWireframe(ObjectData& od) {
    // TODO(jg)
    
  }

  // Utility
  static Point getPointFromNDCVertex(Vertex& v) {
    int x = round((v.x + 1) * (width / 2));
    int y = round((v.y + 1) * (height / 2));
    return Point{x, y};
  }
  
}

#endif // CANVAS_H
