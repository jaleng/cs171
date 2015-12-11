#ifndef ANIMATION_H_
#define ANIMATION_H_

#include <Eigen/Dense>
#include "Quaternion.h"
#include <iostream>
#include <vector>
#include "Transforms.h"

/** Matrix to make Catmull-Rom splines **/
Eigen::Matrix<double, 4, 4> catmull_rom_B;

/** Initialize Catmull-Rom matrix **/
void initialize_catmull_rom_B() {
  catmull_rom_B << 0, 2, 0, 0,
                  -1, 0, 1, 0,
                   2, -5, 4, -1,
                  -1, 3, -3, 1;
  catmull_rom_B /= 2;
}

/** Holds transforms for a frame **/
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

/** Function to wrap indices after incrementing **/
int wrap(int i, int size) {
  while (i < 0) {
    i += size;
  }
  return i % size;
}

/** Holds transformations for keyframes and inbetweens **/
class Animation {
 public:
  /** list of keyframes indices **/
  std::vector<int> keyframes;
  /** Total number of frames of animation (keyframes + inbetweens) **/
  int number_frames;
  /** Transforms for each frame **/
  std::vector<Frame_Transform> ft;

  // ctor
  explicit Animation(int _number_frames)
    : number_frames{_number_frames},
    ft{static_cast<size_t>(_number_frames)}
    {}

  /** Interpolate scaling transformations using Catmull-Rom splines **/
  void interpolate_scaling() {
    using Eigen::Matrix;
    for (auto i = 0; i < keyframes.size(); ++i) {
      // Get the indices of the keyframes to be used to make cardinal spline
      int im1 = wrap((i - 1), keyframes.size());
      int ip1 = wrap((i + 1), keyframes.size());
      int ip2 = wrap((i + 2), keyframes.size());
      // Get frame numbers
      int fim1 = keyframes[im1];
      int fi   = keyframes[i  ];
      int fip1 = keyframes[ip1];
      int fip2 = keyframes[ip2];
      if (fi == fip1) {
        continue;
      }
      // Build p matrix
      Matrix<double, 4, 3> p;
      p << ft[fim1].scale.scaling[0], ft[fim1].scale.scaling[1], ft[fim1].scale.scaling[2],
           ft[fi  ].scale.scaling[0], ft[fi  ].scale.scaling[1], ft[fi  ].scale.scaling[2],
           ft[fip1].scale.scaling[0], ft[fip1].scale.scaling[1], ft[fip1].scale.scaling[2],
           ft[fip2].scale.scaling[0], ft[fip2].scale.scaling[1], ft[fip2].scale.scaling[2];
      Matrix<double, 4, 3> Bp;
      Bp = catmull_rom_B * p;
      double du = 0;
      if (keyframes[i] < fip1) {
        du = 1.0 / (fip1 - keyframes[i]);
      } else {
        du = 1.0 / (fip1 + number_frames - keyframes[i]);
      }

      double u = du;
      for (int inbetween = (keyframes[i] + 1) % number_frames;
           inbetween != fip1;
           inbetween = (inbetween + 1) % number_frames) {
        Matrix<double, 1, 4> u_m;
        u_m << 1.0, u, u*u, u*u*u;
        Matrix<double, 1, 3> new_scale;
        new_scale = u_m*Bp;
        // Set the inbetween to have the interpolated transformation
        ft[inbetween].scale.scaling[0] = new_scale(0, 0);
        ft[inbetween].scale.scaling[1] = new_scale(0, 1);
        ft[inbetween].scale.scaling[2] = new_scale(0, 2);
        u += du;
      }
    }
  }

  /** Interpolate translations using Catmull-Rom splines **/
  void interpolate_translation() {
    using Eigen::Matrix;
    for (auto i = 0; i < keyframes.size(); ++i) {
      int im1 = wrap((i - 1), keyframes.size());
      int ip1 = wrap((i + 1), keyframes.size());
      int ip2 = wrap((i + 2), keyframes.size());
      // Get frame numbers
      int fim1 = keyframes[im1];
      int fi   = keyframes[i  ];
      int fip1 = keyframes[ip1];
      int fip2 = keyframes[ip2];
      if (fi == fip1) {
        continue;
      }
      // Build p matrix
      Matrix<double, 4, 3> p;
      p << ft[fim1].translation.translation[0], ft[fim1].translation.translation[1], ft[fim1].translation.translation[2],
           ft[fi  ].translation.translation[0], ft[fi  ].translation.translation[1], ft[fi  ].translation.translation[2],
           ft[fip1].translation.translation[0], ft[fip1].translation.translation[1], ft[fip1].translation.translation[2],
           ft[fip2].translation.translation[0], ft[fip2].translation.translation[1], ft[fip2].translation.translation[2];
      Matrix<double, 4, 3> Bp;
      Bp = catmull_rom_B * p;
      double du = 0;
      if (keyframes[i] < fip1) {
        du = 1.0 / (fip1 - keyframes[i]);
      } else {
        du = 1.0 / (fip1 + number_frames - keyframes[i]);
      }

      double u = du;
      for (int inbetween = (keyframes[i] + 1) % number_frames;
           inbetween != fip1;
           inbetween = (inbetween + 1) % number_frames) {
        Matrix<double, 1, 4> u_m;
        u_m << 1.0, u, u*u, u*u*u;
        Matrix<double, 1, 3> new_translation;
        new_translation = u_m*Bp;
        // Set the inbetween to have the interpolated transformation
        ft[inbetween].translation.translation[0] = new_translation(0, 0);
        ft[inbetween].translation.translation[1] = new_translation(0, 1);
        ft[inbetween].translation.translation[2] = new_translation(0, 2);
        u += du;
      }
    }
  }

  /** Interpolate rotations using SLERP **/
  void interpolate_rotation() {
    using Eigen::Matrix;
    for (auto i = 0; i < keyframes.size(); ++i) {
      int ip1 = wrap((i + 1), keyframes.size());
      int fi = keyframes[i];
      int fip1 = keyframes[ip1];
      if (fi == fip1) {
        continue;
      }
      // Get q1
      auto q1 = Quaternion{ft[fi].rotation.rotation[0],
                           ft[fi].rotation.rotation[1],
                           ft[fi].rotation.rotation[2],
                           ft[fi].rotation.rotation_angle,
                           true};

      // Get q2
      auto q2 = Quaternion{ft[fip1].rotation.rotation[0],
                           ft[fip1].rotation.rotation[1],
                           ft[fip1].rotation.rotation[2],
                           ft[fip1].rotation.rotation_angle,
                           true};
      double du = 0;
      if (fi < fip1) {
        du = 1.0 / (fip1 - fi);
      } else {
        du = 1.0 / (fip1 + number_frames - fi);
      }

      double u = du;
      double omega = acos(q1.dot(q2));
      for (int inbetween = (fi + 1) % number_frames;
           inbetween != fip1;
           inbetween = (inbetween + 1) % number_frames) {
        Quaternion qu;
        // if sin(omega) close to zero, use the limit formula sin(omega)->0
        // to prevent numerical error
        if (sin(omega) > .001) {
          qu = ((sin((1-u)*omega)/sin(omega))*q1)
               + ((sin(u*omega)/sin(omega)) * q2);
        } else {
          qu = ((1-u) * q1) + (u * q2);
        }
        auto rotation = qu.getRotation();
        ft[inbetween].rotation = rotation;
        u += du;
      }
    }
  }

  /** Interpolate translations, scalings, rotations **/
  void interpolate() {
    interpolate_scaling();
    interpolate_translation();
    interpolate_rotation();
  }

  /** Print transformations. For debugging purposes **/
  void print() {
    using std::cout;
    using std::endl;
    for (int f = 0; f < number_frames; ++f) {
      cout << "Frame" << f << "\t" << endl;
      cout << "t" << "\t" << ft[f].translation.translation[0] << "\t" << ft[f].translation.translation[1] << "\t" << ft[f].translation.translation[2] << endl;
      cout << "s" << "\t" << ft[f].scale.scaling[0] << "\t" << ft[f].scale.scaling[1] << "\t" << ft[f].scale.scaling[2] << endl;
      cout << "rot" << "\t" << ft[f].rotation.rotation[0] << "\t" << ft[f].rotation.rotation[1] << "\t" << ft[f].rotation.rotation[2] << "\t" << ft[f].rotation.rotation_angle << endl;
    }
  }
};

#endif  // ANIMATION_H_
