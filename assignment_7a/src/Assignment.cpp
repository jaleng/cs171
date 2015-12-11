#include "Assignment.hpp"

#include "Utilities.hpp"

#include "Scene.hpp"
#include "UI.hpp"
#include "model.hpp"

#include <vector>
#include <memory>

/** Take a Transform and create a transformation matrix **/
Matrix<double, 4, 4> tfm2mat(const Transformation& tfm) {
  Matrix<double, 4, 4> mat;
  auto x = tfm.trans[0];
  auto y = tfm.trans[1];
  auto z = tfm.trans[2];
  auto w = tfm.trans[3];

  switch(tfm.type) {
  case TRANS:
    assert(w == 1);
    mat << 1, 0, 0, x,
           0, 1, 0, y,
           0, 0, 1, z,
           0, 0, 0, 1;
    break;
  case SCALE:
    assert(w == 1);
    mat << x, 0, 0, 0,
           0, y, 0, 0,
           0, 0, z, 0,
           0, 0, 0, 1;
    break;
  case ROTATE:    // Normalize
    auto length = sqrt(x * x + y * y + z * z);
    x /= length;
    y /= length;
    z /= length;

    auto theta = w;
    auto ct = cos(theta);
    auto st = sin(theta);
    auto x2 = x * x;
    auto y2 = y * y;
    auto z2 = z * z;
    auto x_y = x * y;
    auto x_z = x * z;
    auto y_z = y * z;
    auto omct = 1 - ct;  // 1 - cos(theta);

    mat <<
      x2  + (1 - x2) * ct, x_y * omct - z * st, x_z * omct + y * st, 0,
      x_y * omct + z * st, y2 + (1 - y2) * ct, y_z * omct - x * st, 0,
      x_z * omct - y * st, y_z * omct + x * st, z2 + (1 - z2) * ct, 0,
      0, 0, 0, 1;

    break;
  }
  return mat;
}

/** Take a vector of transforms and create a transformation matrix **/
Matrix<double, 4, 4> tfmvec2mat(const vector<Transformation>& tfmvec) {
  Matrix<double, 4, 4> mat;
  mat.setIdentity();
  for (auto it = tfmvec.crbegin(); it != tfmvec.crend(); ++it) {
    auto tmp = mat;
    mat = tfm2mat(*it) * tmp;
  }
  return mat;
}

/** Superquadric inside-outside test **/
double sq_io(double x, double y, double z, double e, double n) {
  return   pow(pow(x*x, 1.0/e) + pow(y*y, 1.0/e), e/n)
         + pow(z*z, 1/n)
         - 1;
}

struct PAT{
  Primitive prm;
  Matrix<double, 4, 4> tfm;
  PAT(const Primitive& _prm, const Matrix<double, 4, 4> _tfm)
    : prm{_prm}, tfm{_tfm}{}
};

// TODO: build vector of primitives traversing tree
std::unique_ptr<vector<PAT>> buildPATs (const Renderable& root, int level=0) {
  auto v = std::make_unique<vector<PAT>>();
  if (level > 20) {
    return std::move(v);
  }
  switch(root.getType()) {
  case PRM:
    {
    //// TODO: case when renderable is primitive
    Matrix<double, 4, 4> m;
    m.setIdentity();
    v->emplace_back(dynamic_cast<const Primitive&>(root), m);
    break;
    }
  case OBJ:
    {
    //// TODO: case when renderable is Object
    auto obj = dynamic_cast<const Object&>(root);
    auto overall_tfm = tfmvec2mat(obj.getOverallTransformation());
    for (const auto& item : obj.getChildren()) {
      auto child = item.second;
      auto child_tfm = tfmvec2mat(child.transformations);
      auto child_made_pat_vec = buildPATs(*Renderable::get(child.name), level + 1);
      for (auto& child_made_pat : *child_made_pat_vec) {
        v->emplace_back(child_made_pat.prm, overall_tfm * child_tfm * child_made_pat.tfm);
      }
    }
    }
    break;
  case MSH:
    assert(false);
    break;
  }
  return std::move(v);
}

void Assignment::drawIOTest() {
  // TODO: Build vector of primitives by traversing tree

  for(int i = -10; i <= 10; ++i) {
    for (int j = -10; j <= 10; ++j) {
      for (int k = -10; k <= 10; ++k) {
        // Do stuff for each (i,j,k)

        /* TODO:
          inside = false
          For each primitive:
            if point inside primitive:
              inside = true;
              draw red point
              break;
          if not inside:
            draw blue point
         */
      }
    }
  }

}

void Assignment::drawIntersectTest(Camera *camera) {

}
