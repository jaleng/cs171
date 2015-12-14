#include "Assignment.hpp"

#include "Utilities.hpp"

#include "Scene.hpp"
#include "UI.hpp"
#include "model.hpp"

#include <vector>
#include <memory>
#include <algorithm>
#include <limits>
#include <iostream>

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
  for (auto it = tfmvec.cbegin(); it != tfmvec.cend(); ++it) {
    auto tmp = mat;
    mat = tfm2mat(*it) * tmp;
  }
  return mat;
}

/** Take a vector of transforms and create a transformation matrix **/
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

struct PAT {
  Primitive prm;
  Matrix<double, 4, 4> tfm;
  Matrix<double, 4, 4> twot;
  PAT(const Primitive& _prm, const Matrix<double, 4, 4> _tfm,
      const Matrix<double, 4, 4> _twot)
    : prm{_prm}, tfm{_tfm}, twot{_twot}{}
};

// draw blue dot at (x,y,z)
void draw_blue_sphere(double x, double y, double z) {
  const float blue_light[3] = {0.0, 0.0, 1.0};

  glMaterialfv(GL_FRONT, GL_AMBIENT, blue_light);
  glPushMatrix();
  auto jitter = .03;
  auto dx = (jitter / 2) - jitter * (rand() / RAND_MAX);
  auto dy = (jitter / 2) - jitter * (rand() / RAND_MAX);
  auto dz = (jitter / 2) - jitter * (rand() / RAND_MAX);
  glTranslated(x + dx, y + dy, z + dz);
  glutSolidSphere(0.03, 5, 5);
  glPopMatrix();
}

// draw red dot at (x,y,z)
void draw_red_sphere(double x, double y, double z) {
  const float red_light[3] = {1.0, 0.0, 0.0};
  glMaterialfv(GL_FRONT, GL_AMBIENT, red_light);
  glPushMatrix();
  auto jitter = .03;
  auto dx = (jitter / 2) - jitter * (rand() / RAND_MAX);
  auto dy = (jitter / 2) - jitter * (rand() / RAND_MAX);
  auto dz = (jitter / 2) - jitter * (rand() / RAND_MAX);
  glTranslated(x + dx, y + dy, z + dz);
  glutSolidSphere(0.03, 5, 5);
  glPopMatrix();
}

/** Build vector of PATs by traversing tree **/
std::unique_ptr<vector<PAT>> buildPATs(const Renderable& root, int level = 0) {
  auto v = std::make_unique<vector<PAT>>();
  if (level > 20) {
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

  for (int i = -10; i <= 10; ++i) {
    for (int j = -10; j <= 10; ++j) {
      for (int k = -10; k <= 10; ++k) {
        // Do stuff for each (i,j,k)
        auto di = static_cast<double>(i);
        auto dj = static_cast<double>(j);
        auto dk = static_cast<double>(k);
        auto inside =  false;
        for (const auto& pat : *pats) {
          // get x,y,z by transforming i,j,k
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

// Assumes discriminant > 0
double tminus(double a, double b, double c) {
  auto disc = b*b - 4*a*c;
  return (-b - sqrt(disc)) / (2*a);
}


// Assumes discriminant > 0
double tplus(double a, double b, double c) {
  auto disc = b*b-4*a*c;
  return (2*c) / (-b - sqrt(disc));
}

// may not be normalized
Vector3d grad_sq_io(double x, double y, double z, double e, double n) {
  Vector3d res;
  auto blah = pow(pow(x*x, 1/e) + pow(y*y, 1/e),
                  e/n - 1);
  res(0) = (2*x*pow(x*x, 1/e-1)*blah) / n;
  res(1) = (2*y*pow(y*y, 1/e-1)*blah) / n;
  res(2) = 2*z*pow(z*z, 1/n-1) / n;
  return res;
}

struct MissOrHit {
  bool hit;
  double t;
  MissOrHit(bool _hit, double _t) : hit{_hit}, t{_t}{}
};

MissOrHit findIntersection(double e, double n,
                           Vector3d a, Vector3d b, double t_old) {
  auto ray = [a, b](double t) {
    return a*t + b;
  };

  auto sq_io_r = [a, b, e, n](Vector3d r) {
    return sq_io(r(0), r(1), r(2), e, n);
  };

  auto grad_sq_io_r = [e, n](Vector3d r) {
    return grad_sq_io(r(0), r(1), r(2), e, n);
  };

  auto g = [a, b, e, n, sq_io_r, ray](double t) {
    return sq_io_r(ray(t));
  };

  auto gp = [a, b, e, n, grad_sq_io_r, ray](double t) {
    return a.dot(grad_sq_io_r(ray(t)));
  };

  auto gp_was_negative = gp(t_old) < 0;
  for (int iteration = 0; iteration < 10000; ++iteration) {
    auto gpt = gp(t_old);
    auto gt = g(t_old);

    if (abs(gt) < 0.00001) {
      return MissOrHit(true, t_old);
    } else if (abs(gpt) < 0.00001){
      return MissOrHit(false, 0);
    } else if (gp_was_negative && gpt > 0) {
      // gp sign changed from negative to positive
      return MissOrHit(false, 0);
    }
    gp_was_negative = gpt < 0;
    t_old = t_old - (gt / gpt);
  }
  assert(false);  // Did not converge or return miss
}

// DEBUG
void printMatrix(MatrixXd m) {
  for (int r = 0; r < m.rows(); r++) {
    std::cout << "row " << r << ": ";
    for (int c = 0; c < m.cols(); c++) {
      std::cout << m(r, c) <<  "\t";
    }
    std::cout << "\n";
  }
}

void Assignment::drawIntersectTest(Camera *camera) {
  // DEBUG:
  std::cout << "JG: Enter drawIntersectTest\n";
  // ENDEBUG:
  /*
    for each sq in the scene:
      test if it hits the sq
      track the intersection closest to the camera
      
    draw line from closest intersection in direction of normal
  */
  // Get b (camera position)
  // Vector3f
  auto b_v = camera->getPosition();
  auto a_v = camera->getAxis();
  Matrix<double, 4, 1> a0_mat;
  a0_mat << 0, 0, -1, 1;

  // Apply camera rotation to a0 to get a_v
  auto axis = camera->getAxis();
  auto angle = camera->getAngle();
  auto rot_mat = tfm2mat(Transformation(
                           ROTATE, axis(0), axis(1), axis(2), angle));
  Matrix<double, 4, 1> a_mat;
  a_mat = rot_mat * a0_mat;
  a_v = Vector3f(a_mat(0), a_mat(1), a_mat(2));

  // DEBUG:
  std::cout << "JG: Camera pos:" << b_v(0) << " " << b_v(1) << " "<< b_v(2) << "\n";
  std::cout << "JG: Camera axs:" << a_v(0) << " " << a_v(1) << " "<< a_v(2) << "\n";
  // ENDEBUG:

  double lowest_t = std::numeric_limits<double>::infinity();
  PAT* closest_pat = nullptr;

  // Build vector of PATs by traversing tree
  auto pats = getpats();
  for (auto& pat : *pats) {
    //// TODO: test if primitive is hit by camera look ray

    // Transform a_v and b_v using the primitive's transform
    Matrix<double, 4, 1> am;
    am << a_v(0), a_v(1), a_v(2), 1;
    Matrix<double, 4, 1> bm;
    bm << b_v(0), b_v(1), b_v(2), 1;

    auto atm = pat.tfm.inverse() * am;
    Vector3d at(atm(0), atm(1), atm(2));
    auto btm = pat.tfm.inverse() * bm;
    Vector3d bt(btm(0), btm(1), btm(2));

    auto a = a_v.dot(a_v);
    auto b = 2*(a_v.dot(b_v));
    auto c = b_v.dot(b_v) - 3.0;

    auto discriminant = b*b - 4*a*c;
    if (discriminant < 0) {
      continue;
    }

    auto tp = tplus(a, b, c);
    auto tm = tminus(a, b, c);

    if (tp < 0 && tm < 0) {
      continue;
    } else if (tp > 0 && tm > 0) {
      auto tmc = findIntersection(pat.prm.getExp0(),
                                  pat.prm.getExp1(),
                                  at,
                                  bt,
                                  tm);
      //assert(tmc.hit == true);
      if (tmc.hit == true and tmc.t < lowest_t) {
        lowest_t = tmc.t;
        closest_pat = &pat;
      }
    } else {
      // Find intersection using tp
      auto tpc = findIntersection(pat.prm.getExp0(),
                                  pat.prm.getExp1(),
                                  at,
                                  bt,
                                  tp);

      // Find intersection using tm
      auto tmc = findIntersection(pat.prm.getExp0(),
                                  pat.prm.getExp1(),
                                  at,
                                  bt,
                                  tm);

      if (tpc.hit && tmc.hit && tpc.t > 0 && tmc.t > 0) {
        // use lowest tc
        auto tf = min(tpc.t, tmc.t);
        if (tf < lowest_t) {
          lowest_t = tf;
          closest_pat = &pat;
        }
      } else if (tpc.hit && tpc.t > 0) {
        if (tpc.t < lowest_t) {
          lowest_t = tpc.t;
          closest_pat = &pat;
        }
        continue;
      } else if (tmc.hit && tmc.t > 0) {
        if (tmc.t < lowest_t) {
          lowest_t = tmc.t;
          closest_pat = &pat;
        }
      }
    }
  }
  if (closest_pat != nullptr) {
    // Get the normal, apply inverse transform (normal form), then draw line
    Matrix<double, 4, 1> am;
    am << a_v(0), a_v(1), a_v(2), 1;
    Matrix<double, 4, 1> bm;
    bm << b_v(0), b_v(1), b_v(2), 1;

    auto atm = closest_pat->tfm.inverse() * am;
    Vector3f at(atm(0), atm(1), atm(2));
    auto btm = closest_pat->tfm.inverse() * bm;
    Vector3f bt(btm(0), btm(1), btm(2));

    std::cout << "JG: atm:\n";
    printMatrix(atm);

    std::cout << "JG: btm:\n";
    printMatrix(btm);

    auto v = at * lowest_t + bt;
    Matrix<double, 4, 1> v_m;
    v_m << v(0), v(1), v(2), 1;
    // DEBUG
    std::cout << "JG: v_m : Intersection before tfm back to world space:\n";
    printMatrix(v_m);
    std::cout << "JG: pat.tfm: \n";
    printMatrix(closest_pat->tfm);
    // ENDEBUG
    auto nt = closest_pat->prm.getNormal(v);
    Matrix<double, 4, 1> nt4;
    nt4 << nt(0), nt(1), nt(2), 1;
    // Transform normal back into world space
    auto n4 = (closest_pat->twot).transpose() * nt4;
    Vector3d n3(n4(0), n4(1), n4(2));
    n3 /= n3.norm();


    // TODO: get intersection point:
    auto i4 = closest_pat->tfm * v_m;
    std::cout << "JG: i4:\n";
    printMatrix(i4);

    auto w = i4(3);
    Vector3d intersection_point(i4(0)/w, i4(1)/w, i4(2)/w);
    //auto intersection_point = Matrix()closest_pat->tfm*v_m;//a_v * lowest_t + b_v;
    auto end_line = intersection_point + n3;

    // DEBUG:
    std::cout << "JG: Drawing a line\n";
    std::cout << "JG: From "
              <<intersection_point(0) << " "
              <<intersection_point(1)  << " "
              <<intersection_point(2) <<"\n";
    std::cout << "JG: To   "
              <<end_line(0) << " "
              <<end_line(1)  << " "
              <<end_line(2) <<"\n";
    // ENDEBUG:
    // TODO: Draw line from intersection point in direction of normal
    // DEBUG
    // Draw red sphere at intersection,
    draw_red_sphere(intersection_point(0), intersection_point(1), intersection_point(2));
    // blue sphere at end of normal line
    draw_blue_sphere(end_line(0), end_line(1), end_line(2));
    // ENDEBUG
    const float purple[3] {1.0, 0.0, 1.0};
    glMaterialfv(GL_FRONT, GL_AMBIENT, purple);
    glBegin(GL_LINES);
    glVertex3f(intersection_point(0), intersection_point(1),
               intersection_point(2));
    glVertex3f(end_line(0), end_line(1), end_line(2));
    glEnd();
  }
}
