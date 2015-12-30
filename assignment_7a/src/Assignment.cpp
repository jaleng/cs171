#include <vector>
#include <memory>
#include <algorithm>
#include <limits>
#include <iostream>
#include <string>

#include "Assignment.hpp"

#include "Utilities.hpp"

#include "Scene.hpp"
#include "UI.hpp"
#include "model.hpp"

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
  PAT(const Primitive& _prm, const Matrix<double, 4, 4> _tfm)
    : prm{_prm}, tfm{_tfm} {}
};

/** Draw a sphere at (x,y,z) of a given color with small jitter **/
void draw_sphere(double x, double y, double z, const float color[3]) {
  glMaterialfv(GL_FRONT, GL_AMBIENT, color);
  glPushMatrix();
  constexpr float jitter = .03;
  // Let deltas be in range [-jitter/2, jitter/2]
  auto dx = (jitter / 2) - jitter * (rand() / RAND_MAX);
  auto dy = (jitter / 2) - jitter * (rand() / RAND_MAX);
  auto dz = (jitter / 2) - jitter * (rand() / RAND_MAX);
  glTranslated(x + dx, y + dy, z + dz);
  glutSolidSphere(0.03, 5, 5);
  glPopMatrix();
}

/** Draw blue dot at (x,y,z), with small jitter **/
void draw_blue_sphere(double x, double y, double z) {
  constexpr float blue_light[3] = {0.0, 0.0, 1.0};
  draw_sphere(x, y, z, blue_light);
}

/** Draw red dot at (x,y,z), with small jitter **/
void draw_red_sphere(double x, double y, double z) {
  const float red_light[3] = {1.0, 0.0, 0.0};
  draw_sphere(x, y, z, red_light);
}

/** Build vector of PATs by traversing tree **/
std::unique_ptr<vector<PAT>> buildPATs(const Renderable& root, int level = 0) {
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
    v->emplace_back(dynamic_cast<const Primitive&>(root), m);
    break;
    }
  case OBJ:
    {
      auto obj = dynamic_cast<const Object&>(root);
      auto overall_tfm = tfmvec2mat(obj.getOverallTransformation());
      for (const auto& item : obj.getChildren()) {
        auto child = item.second;
        auto child_tfm = tfmvec2mat(child.transformations);
        auto child_made_pat_vec = buildPATs(*Renderable::get(child.name),
                                            level + 1);
        for (auto& child_made_pat : *child_made_pat_vec) {
          v->emplace_back(child_made_pat.prm,
                          overall_tfm * child_tfm * child_made_pat.tfm);
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

std::unique_ptr<vector<PAT>> getpats() {
  const Line* cur_state = CommandLine::getState();
  Renderable* ren = nullptr;
  if (cur_state) {
    ren = Renderable::get(cur_state->tokens[1]);
    return std::move(buildPATs(*ren));
  } else {
    return std::make_unique<vector<PAT>>();
  }
}

Matrix<double, 4, 4> getprmtfmmat(const Primitive& prm) {
  auto coeff = prm.getCoeff();
  Transformation t{SCALE, coeff[0], coeff[1], coeff[2], 1};
  return tfm2mat(t);
}

void Assignment::drawIOTest() {
  // Build vector of PATs by traversing tree
  auto pats = getpats();

  // (i/2, j/2, k/2) is the point in world space we are testing
  // inside the for loops
  for (int i = -10; i <= 10; ++i) {
    for (int j = -10; j <= 10; ++j) {
      for (int k = -10; k <= 10; ++k) {
        auto di = static_cast<double>(i);
        auto dj = static_cast<double>(j);
        auto dk = static_cast<double>(k);
        auto inside =  false;
        for (const auto& pat : *pats) {
          // get x,y,z by transforming i/2,j/2,k/2
          Matrix<double, 4, 1> pretfm;
          pretfm << di /2.0, dj / 2.0, dk/2.0, 1;
          auto posttfm = (pat.tfm * getprmtfmmat(pat.prm)).inverse() * pretfm;
          auto w = posttfm(3);
          auto x = posttfm(0) / w;
          auto y = posttfm(1) / w;
          auto z = posttfm(2) / w;

          if (sq_io(x, y, z, pat.prm.getExp0(), pat.prm.getExp1()) <= 0) {
            inside = true;
            break;
          }
        }
        if (inside) {
          draw_red_sphere(di/2.0, dj/2.0, dk/2.0);
        } else {
          draw_blue_sphere(di/2.0, dj/2.0, dk/2.0);
        }
      }
    }
  }
}

/** Finds initial guess t- for intersection finding algorithm
 *  Assumes discriminant > 0
 **/
double tminus(double a, double b, double c) {
  auto disc = b*b - 4*a*c;
  return (-b - sqrt(disc)) / (2*a);
}


/** Finds initial guess t+ for intersection finding algorithm
 *  Assumes discriminant > 0
 **/
double tplus(double a, double b, double c) {
  auto disc = b*b-4*a*c;
  return (2*c) / (-b - sqrt(disc));
}

/** Get gradient of superquadric inside-out function at a given point
 *  May not be normalized.
 *  Possible numerical errors for small e, n.
 **/
Vector3d grad_sq_io(double x, double y, double z, double e, double n) {
  Vector3d res;
  auto term = pow(pow(x*x, 1/e) + pow(y*y, 1/e),
                  e/n - 1);
  res(0) = (2*x*pow(x*x, 1/e-1)*term) / n;
  res(1) = (2*y*pow(y*y, 1/e-1)*term) / n;
  res(2) = 2*z*pow(z*z, 1/n-1) / n;
  return res;
}

/** Maybe double. If hit is true, t is valid; otherwise not. **/
struct MissOrHit {
  bool hit;
  double t;
  MissOrHit(bool _hit, double _t) : hit{_hit}, t{_t} {}
  explicit MissOrHit(bool _hit) : hit{false}, t{0} {
    assert(_hit == false);
  }
  explicit MissOrHit(double _t) : hit{true}, t{_t} {}
};

MissOrHit findIntersection(double e, double n,
                           Vector3d a, Vector3d b, double t_old) {
  // function to get ray from t
  auto ray = [a, b](double t) {
    return a*t + b;
  };
  // function to get sq_io from ray r
  auto sq_io_r = [a, b, e, n](Vector3d r) {
    return sq_io(r(0), r(1), r(2), e, n);
  };
  // function to get grad_sq_io from ray r
  auto grad_sq_io_r = [e, n](Vector3d r) {
    return grad_sq_io(r(0), r(1), r(2), e, n);
  };

  auto g = [a, b, e, n, sq_io_r, ray](double t) {
    return sq_io_r(ray(t));
  };
  // derivative of g
  auto gp = [a, b, e, n, grad_sq_io_r, ray](double t) {
    return a.dot(grad_sq_io_r(ray(t)));
  };

  auto gp_was_negative = gp(t_old) < 0;
  constexpr int max_iterations = 1000;
  for (int iteration = 0; iteration < max_iterations; ++iteration) {
    auto gpt = gp(t_old);
    auto gt = g(t_old);

    if (abs(gt) < 0.00001) {
      return MissOrHit(t_old);
    } else if (abs(gpt) < 0.00001) {
      return MissOrHit(false);
    } else if (gp_was_negative && gpt > 0) {
      // gp sign changed from negative to positive
      return MissOrHit(false);
    }
    gp_was_negative = gpt < 0;
    t_old = t_old - (gt / gpt);
  }
  std::cout << "findIntersect did not converge; exp values may be too small.\n";
  assert(false);  // Did not converge or return miss
}

/** For DEBUG, prints out a matrix row-by-row **/
void printMatrix(MatrixXd m, std::string msg = "") {
  if (!msg.empty()) {
    std::cout << msg;
  }
  for (int r = 0; r < m.rows(); r++) {
    std::cout << "row " << r << ": ";
    for (int c = 0; c < m.cols(); c++) {
      std::cout << m(r, c) <<  "\t";
    }
    std::cout << "\n";
  }
}


Vector3d transform(Vector3d v, Matrix<double, 4, 4> tfm) {
  Vector4d v4;
  v4 << v(0), v(1), v(2), 1;
  auto tmp = tfm * v4;
  return Vector3d(tmp(0)/tmp(3), tmp(1)/tmp(3), tmp(2)/tmp(3));
}



PAT* get_closest_PAT_thru_ray(vector<PAT>& pats,
                              Vector3d A,
                              Vector3d B,
                              double *t_save = nullptr) {
  // Iterating throught the pats, we will find the closest intersection
  double lowest_t = std::numeric_limits<double>::infinity();
  PAT* closest_pat = nullptr;

  for (auto& pat : pats) {
    auto B_transformed = transform(B, (pat.tfm
                                       * getprmtfmmat(pat.prm)).inverse());
    auto BplusA_transformed =
      transform(B+A, (pat.tfm * getprmtfmmat(pat.prm)).inverse());
    auto A_transformed = BplusA_transformed - B_transformed;

    // Prepare for intersect finding

    auto a = A_transformed.dot(A_transformed);
    auto b = 2*(A_transformed.dot(B_transformed));
    auto c = B_transformed.dot(B_transformed) - 3.0;

    auto discriminant = b*b - 4*a*c;
    if (discriminant < 0) {
      // No intersection
      continue;
    }

    auto tp = tplus(a, b, c);
    auto tm = tminus(a, b, c);

    if (tp < 0 && tm < 0) {
      // sq is behind camera, no intersection
      continue;
    } else if (tp > 0 && tm > 0) {
      // sq bounding box is in front of camera
      auto tmc = findIntersection(pat.prm.getExp0(),
                                  pat.prm.getExp1(),
                                  A_transformed,
                                  B_transformed,
                                  tm);
      if (tmc.hit == true && tmc.t < lowest_t) {
        // Found a new closest intersection
        lowest_t = tmc.t;
        closest_pat = &pat;
      }
    } else {
      // Find intersection using tp
      auto tpc = findIntersection(pat.prm.getExp0(),
                                  pat.prm.getExp1(),
                                  A_transformed,
                                  B_transformed,
                                  tp);

      // Find intersection using tm
      auto tmc = findIntersection(pat.prm.getExp0(),
                                  pat.prm.getExp1(),
                                  A_transformed,
                                  B_transformed,
                                  tm);

      if (tpc.hit && tmc.hit && tpc.t > 0 && tmc.t > 0) {
        // use lowest tc
        auto tf = min(tpc.t, tmc.t);
        if (tf < lowest_t) {
          // Found a new closest intersection
          lowest_t = tf;
          closest_pat = &pat;
        }
      }
    }
  }
  if (t_save != nullptr) {
    *t_save = lowest_t;
  }
  return closest_pat;
}

Vector3d getCameraLook(const Camera& camera) {
  auto axis = camera.getAxis();
  auto angle = camera.getAngle();
  auto rot_mat = tfm2mat(Transformation(
                           ROTATE, axis(0), axis(1), axis(2), angle));
  return transform(Vector3d{0, 0, -1}, rot_mat);
}

Vector3d getCameraPosition(const Camera& camera) {
  return camera.getPosition().cast<double>();
}

Vector3d transformNormal(Vector3d v, Matrix<double, 4, 4> tfm) {
  Matrix<double, 3, 3> tfm3x3;
  tfm3x3 <<
    tfm(0, 0), tfm(0, 1), tfm(0, 2),
    tfm(1, 0), tfm(1, 1), tfm(1, 2),
    tfm(2, 0), tfm(2, 1), tfm(2, 2);
  return tfm3x3.inverse().transpose() * v;
}

void drawProminentLineSegment(Vector3d start_line, Vector3d end_line) {
  draw_red_sphere(start_line(0), start_line(1),
                  start_line(2));
  // Draw blue sphere at end of normal line
  draw_blue_sphere(end_line(0), end_line(1), end_line(2));
  // Draw purple line from intersection point along normal
  const float purple[3] {1.0, 0.0, 1.0};
  glMaterialfv(GL_FRONT, GL_AMBIENT, purple);
  glBegin(GL_LINES);
  glVertex3f(start_line(0), start_line(1),
             start_line(2));
  glVertex3f(end_line(0), end_line(1), end_line(2));
  glEnd();
}

void Assignment::drawIntersectTest(Camera *camera) {
  auto camera_look = getCameraLook(*camera);
  auto camera_position = getCameraPosition(*camera);

  // Build vector of PATs by traversing tree
  auto pats = getpats();
  // Iterating throught the pats, we will find the closest intersection
  double lowest_distance = std::numeric_limits<double>::infinity();
  PAT* closest_pat = nullptr;
  closest_pat = get_closest_PAT_thru_ray(*pats,
                                         camera_look,
                                         camera_position,
                                         &lowest_distance);

  // If there is a closest intersection, draw the normal of the surface
  // at the intersection.
  if (closest_pat != nullptr) {
    // Get the intersection and normal, apply inverse transforms, then draw line
    auto pat = *closest_pat;
    Matrix4d pat_transform_and_scale_matrix = pat.tfm * getprmtfmmat(pat.prm);
    Vector3d camera_position_plus_look_transformed =
      transform(camera_position + camera_look,
                pat_transform_and_scale_matrix.inverse());
    Vector3d camera_position_transformed =
      transform(camera_position,
                pat_transform_and_scale_matrix.inverse());
    Vector3d camera_look_transformed = camera_position_plus_look_transformed
                  - camera_position_transformed;

    auto intersection_transformed =
      camera_position_transformed
      + camera_look_transformed * lowest_distance;

    auto scaled = transform(intersection_transformed, getprmtfmmat(pat.prm));
    Vector3d normal_transformed = pat.prm.getNormal(
                                    scaled.cast<float>()).cast<double>();
    Vector3d normal_world = transformNormal(normal_transformed,
                                            pat_transform_and_scale_matrix);
    normal_world.normalize();

    Vector3d intersection_point = transform(intersection_transformed,
                                            pat_transform_and_scale_matrix);
    auto end_line = intersection_point + normal_world;

    drawProminentLineSegment(intersection_point, end_line);
  }
}
