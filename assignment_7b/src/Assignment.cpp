#include <vector>
#include <memory>
#include <algorithm>
#include <limits>
#include <iostream>
#include <string>

#include "Assignment.hpp"

#include "UI.hpp"
#include "Scene.hpp"

#define XRES 250
#define YRES 250

using std::unique_ptr;
/** Take a Transform and create a transformation matrix **/
Matrix<double, 4, 4> tfm2mat(const Transformation& tfm) {
  Matrix<double, 4, 4> mat;
  auto x = tfm.trans[0];
  auto y = tfm.trans[1];
  auto z = tfm.trans[2];
  auto w = tfm.trans[3];

  switch (tfm.type) {
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
  case ROTATE:
    // Normalize
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
  for (auto it = tfmvec.cbegin(); it != tfmvec.cend(); ++it) {
    auto tmp = mat;
    mat = tfm2mat(*it) * tmp;
  }
  return mat;
}

/** Take a vector of transforms and create a transformation matrix
 *   that does not factor in translations 
 **/
Matrix<double, 4, 4> tfmvec2mat_wo_tl(const vector<Transformation>& tfmvec) {
  Matrix<double, 4, 4> mat;
  mat.setIdentity();
  for (auto it = tfmvec.cbegin(); it != tfmvec.cend(); ++it) {
    auto tmp = mat;
    if (it->type != TRANS) {
      mat = tfm2mat(*it) * tmp;
    }
  }
  return mat;
}

/** Superquadric inside-outside test
 *  < 0 -> inside object
 *  = 0 -> on object's surface
 *  > 0 -> outside the object
 */
double sq_io(double x, double y, double z, double e, double n) {
  return   pow(pow(x*x, 1.0/e) + pow(y*y, 1.0/e), e/n)
         + pow(z*z, 1.0/n)
         - 1.0;
}

/** PAT, structure to hold a primitive and associated transforms.
 *  This type will be referenced as 'pat' in code and comments 
 **/
struct PAT {
  Primitive prm;
  Matrix<double, 4, 4> tfm;
  Matrix<double, 4, 4> twot;
  PAT(const Primitive& _prm, const Matrix<double, 4, 4> _tfm,
      const Matrix<double, 4, 4> _twot)
    : prm{_prm}, tfm{_tfm}, twot{_twot}{}
};
/** Build vector of PATs by traversing tree **/
unique_ptr<vector<PAT>> buildPATs(const Renderable& root, int level = 0) {
  auto v = std::make_unique<vector<PAT>>();
  constexpr int max_depth = 20;
  if (level > max_depth) {
    // Exceeded max depth, do not add any more primitives/objects
    return std::move(v);
  }
  switch (root.getType()) {
  case PRM:
    {
    Matrix<double, 4, 4> m;
    m.setIdentity();
    v->emplace_back(dynamic_cast<const Primitive&>(root), m, m);
    break;
    }
  case OBJ:
    {
    auto obj = dynamic_cast<const Object&>(root);
    auto overall_tfm = tfmvec2mat(obj.getOverallTransformation());
    auto overall_twot = tfmvec2mat_wo_tl(obj.getOverallTransformation());
    for (const auto& item : obj.getChildren()) {
      auto child = item.second;
      auto child_tfm = tfmvec2mat(child.transformations);
      auto child_twot = tfmvec2mat_wo_tl(child.transformations);
      auto child_made_pat_vec = buildPATs(*Renderable::get(child.name), level + 1);
      for (auto& child_made_pat : *child_made_pat_vec) {
        v->emplace_back(child_made_pat.prm,
                        overall_tfm * child_tfm * child_made_pat.tfm,
                        overall_twot * child_twot * child_made_pat.twot);
      }
    }
    }
    break;
  case MSH:
    // I don't know what this is
    assert(false);
    break;
  }
  return std::move(v);
}

unique_ptr<vector<PAT>> getPATs(const Scene& scene) {
  auto pats = std::make_unique<vector<PAT>>();
  for (const auto& obj_p : scene.root_objs) {
    for (const auto& pat :
           *buildPATs(dynamic_cast<const Renderable&>(*obj_p))) {
      pats->push_back(pat);
    }
  }
  return std::move(pats);
}

Vector3d transform(Vector3d v, Matrix<double, 4, 4> tfm) {
  Vector4d v4;
  v4 << v(0), v(1), v(2), 1;
  auto tmp = tfm * v4;
  return Vector3d(tmp(0)/tmp(3), tmp(1)/tmp(3), tmp(2)/tmp(3));
}

Vector3d transformNormal(Vector3d v, Matrix<double, 4, 4> tfm) {
  Vector4d v4;
  v4 << v(0), v(1), v(2), 0;
  auto tmp = tfm.inverse().transpose() * v4;
  return Vector3d(tmp(0), tmp(1), tmp(2));
}

Vector3d getB(const Camera& camera) {
  return camera.getPosition().cast<double>();
}

Vector3d getA(const Camera& camera, int i, int j) {
  // Get width and height of screen plane
  auto height = 2 * camera.getNear()
                  * tan(static_cast<double>(camera.getFov()) / 2.0);
  auto width = static_cast<double>(camera.getAspect()) * height;

  //// Get camera basis vectors
  // Get camera rotation
  auto axis = camera.getAxis();
  auto angle = camera.getAngle();
  auto rot_mat = tfm2mat(Transformation(
                           ROTATE, axis(0), axis(1), axis(2), angle));
  auto look  = transform(Vector3d(0, 0, -1), rot_mat);
  auto right = transform(Vector3d(1, 0, 0), rot_mat);
  auto up    = transform(Vector3d(0, 1, 0), rot_mat);

  auto x_i = [width](int i) {return (i - XRES/2.0) * width / XRES;};
  auto y_j = [height](int j) {return (j - YRES/2.0) * height / YRES;};

  return  (look * camera.getNear()) + (right * x_i(i)) + (up * y_j(j));
}

Vector3d lighting(Vector3d lit_pos, Vector3d normal,
         const Primitive& prm, const vector<PointLight>& lights,
         Vector3d cam_pos);
bool isShaded(const PointLight& light, Vector3d lit_pos, const Primitive& prm);

/* Ray traces the scene. */
void Assignment::raytrace(Camera camera, Scene scene) {
    // LEAVE THIS UNLESS YOU WANT TO WRITE YOUR OWN OUTPUT FUNCTION
    PNGMaker png = PNGMaker(XRES, YRES);

    // REPLACE THIS WITH YOUR CODE
    for (int i = 0; i < XRES; i++) {
        for (int j = 0; j < YRES; j++) {
            png.setPixel(i, j, 1.0, 1.0, 1.0);
        }
    }

    // LEAVE THIS UNLESS YOU WANT TO WRITE YOUR OWN OUTPUT FUNCTION
    if (png.saveImage())
        printf("Error: couldn't save PNG image\n");
}
