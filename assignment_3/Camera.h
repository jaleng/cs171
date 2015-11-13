#ifndef CAMERA_H_
#define CAMERA_H_

struct Camera {
  float position[3];
  float orientation_axis[3];
  float orientation_angle;
  float near, far, left, right, top, bottom;

  Camera(float px, float py, float pz,
         float ox, float oy, float oz, float angle,
         float _near, float _far,
         float _left, float _right,
         float _top, float _bottom)
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
