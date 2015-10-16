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
    pixels = Eigen::MatrixXd::Zero(width, height);
  }

  // Mutators

  /** "Fill" in a pixel of the pixel grid */
  void fill(int x, int y) {
    pixels(x,y) = 1;
  }

  /** Draw a line on the canvas between the two given points */
  void drawLine(Point p1, Point p2) {
    // Do not draw if one of the points is out of the bounds of the grid
    if (p1.x < 0 || p1.x >= width) {
      // p1.x out of bounds
      return;
    } else if (p1.y < 0 || p1.y >= height) {
      // p1.y out of bounds
      return;
    } else if (p2.x < 0 || p2.x >= width) {
      // p2.x out of bounds
      return;
    } else if (p2.y < 0 || p2.y >= height) {
      // p2.y out of bounds
      return;
    } else {
      // Both points in bounds. Draw line.
      // Get the x and y coords of each point
      int x0, x1, y0, y1;
      if (p1.x < p2.x) {
        x0 = p1.x;
        y0 = p1.y;
        x1 = p2.x;
        y1 = p2.y;
      } else {
        // swap so that x0 < x1
        x0 = p2.x;
        y0 = p2.y;
        x1 = p1.x;
        y1 = p1.y;
      }

      int dy = y1 - y0;
      int dx = x1 - x0;
      int epsilon = 0;
      
      // Line-drawing algorithm works differently depending on the slope.
      // treat m>1, 1 >= m > 0, 0 >= m > -1, m <= -1 cases separately

      if (dy > dx) {
        // m > 1
        int x = x0;
        for (int y = y0; y <= y1; ++y) {
          fill(x, y);
          if (2 * (epsilon + dx) < dy) {
            epsilon = epsilon + dx;
          } else {
            epsilon = epsilon + dx - dy;
            ++x;
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
            ++y;
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
            --y;
          }
        }

      } else {
        // m <= -1
        int x = x0;
        for (int y = y0; y >= y1; --y) {
          fill(x, y);
          if (2 * (epsilon - dx) > dy) {
            epsilon = epsilon - dx;
          } else {
            epsilon = epsilon - dx - dy;
            ++x;
          }
        }
      }
    }
  }

  /** Draw line on the canvas between two points representing the passed vertices.
   *  Vertices must be in NDC form
   */
  void drawLine(Vertex& v1, Vertex& v2) {
    Point p1 = getPointFromNDCVertex(v1);
    Point p2 = getPointFromNDCVertex(v2);
    drawLine(p1, p2);
  }

  /** Draw wire frame of object data to canvas (lines connecting vertices of faces). */
  void drawWireFrame(ObjectData& od) {
    for (auto& face : od.faces) {
      // Draw lines on the canvas connecting the vertices of this face.
      auto v1_idx = face.v1_idx - 1;
      auto v2_idx = face.v2_idx - 1;
      auto v3_idx = face.v3_idx - 1;
      drawLine(od.vertices[v1_idx], od.vertices[v2_idx]);
      drawLine(od.vertices[v1_idx], od.vertices[v3_idx]);
      drawLine(od.vertices[v2_idx], od.vertices[v3_idx]);
    }
  }

  // Utility
  /** Get point to represent NDC vertex in canvas-space. */
  Point getPointFromNDCVertex(Vertex& v) {
    int x = round((v.x + 1) * (width / 2));
    int y = round((-v.y + 1) * (height / 2));
    return Point{x, y};
  }
};

#endif // CANVAS_H
