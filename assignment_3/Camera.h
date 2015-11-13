#ifndef CAMERA_H_
#define CAMERA_H_

struct Camera {
  double position[3];
  double orientation_axis[3];
  double orientation_angle;
  double near, far, left, right, top, bottom;

  Camera(double px, double py, double pz,
         double ox, double oy, double oz, double angle,
         double _near, double _far,
         double _left, double _right,
         double _top, double _bottom)
  : position {px, py, pz},
    orientation_axis {ox, oy, oz},
    orientation_angle {angle},
    near{_near},
    far{_far},
    left{_left},
    right{_right},
    top{_top},
    bottom{_bottom} {};
};

#endif  // CAMERA_H_
