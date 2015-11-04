#ifndef _PERSPECTIVE_H_
#define _PERSPECTIVE_H_

/** Class to hold perspective info (frustrum parameters) */
class Perspective {
 public:
  double near;
  double far;
  double top;
  double bottom;
  double left;
  double right;

  /** Construct perspective object given frustrum parameters */
  explicit Perspective(double _near, double _far, double _left,
                       double _right, double _top, double _bottom)
    : near{_near},
      far{_far},
      top{_top},
      bottom{_bottom},
      left{_left},
      right{_right} {}
};

#endif  // _PERSPECTIVE_H_
