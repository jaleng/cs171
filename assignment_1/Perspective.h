class Perspective {
public:
  double near;
  double far;
  double top;
  double bottom;
  double left;
  double right;

  explicit Perspective(double _near, double _far, double _left,
                       double _right, double _top, double _bottom)
    : near{_near},
      far{_far},
      top{_top},
      bottom{_bottom},
      left{_left},
      right{_right} {}
};
