#ifndef ANIMATION_H_
#define ANIMATION_H_

#include <Eigen/Dense>
#include "Quaternion.h"

Eigen::Matrix<double, 4, 4> catmull_rom_B;
void initialize_catmull_rom_B() {
  catmull_rom_B << 0, 2, 0, 0,
                  -1, 0, 1, 0,
                   2, -5, 4, -1,
                  -1, 3, -3, 1;
  catmull_rom_B /= 2;
}

#include <vector>
#include "Transforms.h"

class Frame_Transform {
 public:
  Transform translation;
  Transform scale;
  Transform rotation;

  Frame_Transform()
    : translation{TransformType::TRANSLATION},
      scale{TransformType::SCALING},
      rotation{TransformType::ROTATION}
      {}
};

int wrap(int a, int b) {
  while (a < 0) {
    a += b;
  }
  return a % b;
}

class Animation {
 public:
  std::vector<int> keyframes;
  int number_frames;
  std::vector<Frame_Transform> ft;

  // ctor
  explicit Animation(int _number_frames)
    : number_frames{_number_frames},
    ft{static_cast<size_t>(_number_frames)}
    {}

  void interpolate_scaling() {
    using Eigen::Matrix;
    for (auto i = 0; i < keyframes.size(); ++i) {
      int im1 = wrap((i - 1), keyframes.size());
      int ip1 = wrap((i + 1), keyframes.size());
      int ip2 = wrap((i + 2), keyframes.size());
      if (keyframes[i] == keyframes[ip1]) {
        continue;
      }
      // Build p matrix
      Matrix<double, 4, 3> p;
      p << ft[keyframes[im1]].scale.scaling[0], ft[keyframes[im1]].scale.scaling[1], ft[keyframes[im1]].scale.scaling[2],
           ft[keyframes[i  ]].scale.scaling[0], ft[keyframes[i  ]].scale.scaling[1], ft[keyframes[i  ]].scale.scaling[2],
           ft[keyframes[ip1]].scale.scaling[0], ft[keyframes[ip1]].scale.scaling[1], ft[keyframes[ip1]].scale.scaling[2],
           ft[keyframes[ip2]].scale.scaling[0], ft[keyframes[ip2]].scale.scaling[1], ft[keyframes[ip2]].scale.scaling[2];
      Matrix<double, 4, 3> Bp;
      Bp = catmull_rom_B * p;
      double du = 0;
      if (keyframes[i] < keyframes[ip1]) {
        du = 1.0 / (keyframes[ip1] - keyframes[i]);
      } else {
        du = 1.0 / (keyframes[i] + number_frames - keyframes[ip1]);
      }

      double u = du;
      for (int inbetween = (keyframes[i] + 1) % number_frames;
           inbetween != keyframes[ip1];
           inbetween = (inbetween + 1) % number_frames) {
        Matrix<double, 1, 4> u_m;
        u_m << 1.0, u, u*u, u*u*u;
        Matrix<double, 1, 3> new_scale;
        new_scale = u_m*Bp;
        ft[inbetween].scale.scaling[0] = new_scale(0, 0);
        ft[inbetween].scale.scaling[1] = new_scale(0, 1);
        ft[inbetween].scale.scaling[2] = new_scale(0, 2);
        u += du;
      }
    }
  }

  void interpolate_translation() {
    using Eigen::Matrix;
    for (auto i = 0; i < keyframes.size(); ++i) {
      int im1 = wrap((i - 1), keyframes.size());
      int ip1 = wrap((i + 1), keyframes.size());
      int ip2 = wrap((i + 2), keyframes.size());
      if (keyframes[i] == keyframes[ip1]) {
        continue;
      }
      // Build p matrix
      Matrix<double, 4, 3> p;
      p << ft[keyframes[im1]].translation.translation[0], ft[keyframes[im1]].translation.translation[1], ft[keyframes[im1]].translation.translation[2],
           ft[keyframes[i  ]].translation.translation[0], ft[keyframes[i  ]].translation.translation[1], ft[keyframes[i  ]].translation.translation[2],
           ft[keyframes[ip1]].translation.translation[0], ft[keyframes[ip1]].translation.translation[1], ft[keyframes[ip1]].translation.translation[2],
           ft[keyframes[ip2]].translation.translation[0], ft[keyframes[ip2]].translation.translation[1], ft[keyframes[ip2]].translation.translation[2];
      Matrix<double, 4, 3> Bp;
      Bp = catmull_rom_B * p;
      double du = 0;
      if (keyframes[i] < keyframes[ip1]) {
        du = 1.0 / (keyframes[ip1] - keyframes[i]);
      } else {
        du = 1.0 / (keyframes[i] + number_frames - keyframes[ip1]);
      }

      double u = du;
      for (int inbetween = (keyframes[i] + 1) % number_frames;
           inbetween != keyframes[ip1];
           inbetween = (inbetween + 1) % number_frames) {
        Matrix<double, 1, 4> u_m;
        u_m << 1.0, u, u*u, u*u*u;
        Matrix<double, 1, 3> new_translation;
        new_translation = u_m*Bp;
        ft[inbetween].translation.translation[0] = new_translation(0, 0);
        ft[inbetween].translation.translation[1] = new_translation(0, 1);
        ft[inbetween].translation.translation[2] = new_translation(0, 2);
        u += du;
      }
    }
  }

  void interpolate_rotation() {
    using Eigen::Matrix;
    for (auto i = 0; i < keyframes.size(); ++i) {
      int ip1 = wrap((i + 1), keyframes.size());
      if (keyframes[i] == keyframes[ip1]) {
        continue;
      }
      // Get q1
      auto q1 = Quaternion{ft[keyframes[i]].rotation.rotation[0],
                           ft[keyframes[i]].rotation.rotation[1],
                           ft[keyframes[i]].rotation.rotation[2],
                           ft[keyframes[i]].rotation.rotation_angle,
                           true};

      // Get q2
      auto q2 = Quaternion{ft[keyframes[ip1]].rotation.rotation[0],
                           ft[keyframes[ip1]].rotation.rotation[1],
                           ft[keyframes[ip1]].rotation.rotation[2],
                           ft[keyframes[ip1]].rotation.rotation_angle,
                           true};

      // for each inbetween
      double du = 0;
      if (keyframes[i] < keyframes[ip1]) {
        du = 1.0 / (keyframes[ip1] - keyframes[i]);
      } else {
        du = 1.0 / (keyframes[i] + number_frames - keyframes[ip1]);
      }

      double u = du;
      double omega = acos(q1.dot(q2));
      for (int inbetween = (keyframes[i] + 1) % number_frames;
           inbetween != keyframes[ip1];
           inbetween = (inbetween + 1) % number_frames) {
        // stuff
        auto qu = ((sin((1-u)*omega)/sin(omega))*q1)
                + ((sin(u*omega)/sin(omega)) * q2);
        auto rotation = qu.getRotation();
        ft[inbetween].rotation = rotation;
        u += du;
      }
    }
  }

  void interpolate() {
    interpolate_scaling();
    interpolate_translation();
    interpolate_rotation();
  }
};

#endif  // ANIMATION_H_
