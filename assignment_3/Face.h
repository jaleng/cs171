#ifndef FACE_H_
#define FACE_H_

/**
 * Hold the indices of the vertices that define a face.
 */
class Face {
public:
  int v1_idx;
  int v2_idx;
  int v3_idx;

  int v1_normal_idx;
  int v2_normal_idx;
  int v3_normal_idx;

  explicit Face(int _v1_idx, int _v2_idx, int _v3_idx,
                int _v1_normal_idx, int _v2_normal_idx, int _v3_normal_idx)
    : v1_idx{_v1_idx},
      v2_idx{_v2_idx},
      v3_idx{_v3_idx},

      v1_normal_idx{_v1_normal_idx},
      v2_normal_idx{_v2_normal_idx},
      v3_normal_idx{_v3_normal_idx} {};
};

#endif // FACE_H_