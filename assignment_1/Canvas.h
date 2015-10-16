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
    pixels = Eigen::MatrixXd::Zero(width, height);
  }

  // Mutators
  void fill(int x, int y) {
    pixels(x,y) = 1;
  }

  void drawLine(Point p1, Point p2) {
    // TODO(jg)
    if (p1.x < 0 || p1.x >= width) {
      return;
    } else if (p1.y < 0 || p1.y >= height) {
      return;
    } else if (p2.x < 0 || p2.x >= width) {
      return;
    } else if (p2.y < 0 || p2.y >= height) {
      return;
    } else {
      // Both points in bounds. Draw line.
      int x0, x1, y0, y1;
      if (p1.x < p2.x) {
        x0 = p1.x;
        y0 = p1.y;
        x1 = p2.x;
        y1 = p2.y;
      } else {
        x0 = p2.x;
        y0 = p2.y;
        x1 = p1.x;
        y1 = p1.y;
      }
      // determine slope >1, >0, >-1,
      int dy = y1 - y0;
      int dx = x1 - x0;
      int epsilon = 0;
      
      if (dy > dx) {
        // m > 1
        int x = x0;
        for (int y = y0; y <= y1; ++y) {
          fill(x, y);
          if (2 * (epsilon + dx) < dy) {
            epsilon = epsilon + dx;
          } else {
            epsilon = epsilon + dx - dy;
            x = x + 1;
          }
        }
      } else if (dy > 0) {
        // 0 < m <= 1
        int y = y0;
        for (int x = x0; x <= x1; ++x) {
          fill(x, y);
          if (2 * (epsilon + dy) < dx) {
            epsilon = epsilon + dy;
          } else {
            epsilon = epsilon + dy - dx;
            y = y + 1;
          }
        }
      } else if (dy > -dx) {
        // -1 < m <= 0
        int y = y0;
        for (int x = x0; x <= x1; ++x) {
          fill(x, y);
          if (2 * (epsilon - dy) < dx) {
            epsilon = epsilon - dy;
          } else {
            epsilon = epsilon - dy - dx;
            y = y - 1;
          }
        }

      } else {
        // m <= -1
        int x = x0;
        for (int y = y0; y >= y1; --y) {
          fill(x, y);
          if (2 * (epsilon - dx) < dy) {
            epsilon = epsilon - dx;
          } else {
            epsilon = epsilon - dx + dy;
            x = x + 1;
          }
        }
      }
    }
  }

  void drawLine(Vertex& v1, Vertex& v2) {
    Point p1 = getPointFromNDCVertex(v1);
    Point p2 = getPointFromNDCVertex(v2);
    drawLine(p1, p2);
  }

  void drawWireFrame(ObjectData& od) {
    for (auto& face : od.faces) {
      auto v1_idx = face.v1_idx - 1;
      auto v2_idx = face.v2_idx - 1;
      auto v3_idx = face.v3_idx - 1;
      drawLine(od.vertices[v1_idx], od.vertices[v2_idx]);
      drawLine(od.vertices[v1_idx], od.vertices[v3_idx]);
      drawLine(od.vertices[v2_idx], od.vertices[v3_idx]);
    }
  }

  // Utility
  Point getPointFromNDCVertex(Vertex& v) {
    int x = round((v.x + 1) * (width / 2));
    int y = round((-v.y + 1) * (height / 2));
    return Point{x, y};
  }
};

#endif // CANVAS_H
