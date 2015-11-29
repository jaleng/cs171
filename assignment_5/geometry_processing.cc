#include "geometry_processing.h"

/** Get normal given HEV **/
Vertex compute_normal(HEV *v) {
  Vertex net_normal;

  auto he = v->out;
  do {
    auto f = he->face;
    Vertex v1 = f->edge->vertex->getVertex();
    Vertex v2 = f->edge->next->vertex->getVertex();
    Vertex v3 = f->edge->next->next->vertex->getVertex();

    // face_normal = cross product of (v2 - v1) x (v3 - v1)
    auto face_normal = cross(v2 - v1, v3 - v1);
    // face area = 1/2 | face_normal |
    auto face_area = 0.5 * face_normal.norm();
    // n += face_normal * face_area
    net_normal += face_normal * face_area;

    he = he->flip->next;
  } while (he != v->out);

  // normalize n and return it
  return net_normal.normalized();
}

/** Cross product **/
Vertex cross(Vertex v1, Vertex v2) {
  Vertex v_res(0, 0, 0);
  v_res.x = v1.y * v2.z - v1.z * v2.y;
  v_res.y = v1.z * v2.x - v1.x * v2.z;
  v_res.z = v1.x * v2.y - v1.y * v2.z;
  return v_res;
}

/** Dot product **/
double dot(Vertex v1, Vertex v2) {
  return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

/** function to assign each vertex in our mesh to an index **/
void index_vertices(std::vector<HEV*> *vertices) {
  for (int i = 1; i < vertices->size(); ++i)  // start at 1 because obj files are 1-indexed
    vertices->at(i)->index = i;  // assign each vertex an index
}

/** Make discrete Laplacian operator **/
// function to construct our B operator in matrix form
Eigen::SparseMatrix<double> build_F_operator(std::vector<HEV*> *vertices, double h) {  
  index_vertices(vertices);  // assign each vertex an index

  // recall that due to 1-indexing of obj files,
  // index 0 of our list doesn't actually contain a vertex  
  int num_vertices = vertices->size() - 1;

  // initialize a sparse matrix to represent our L operator
  Eigen::SparseMatrix<double> L(num_vertices, num_vertices);

  // reserve room for 7 non-zeros per row of L
  L.reserve(Eigen::VectorXi::Constant(num_vertices, 7));

  for (int i = 1; i < vertices->size(); ++i) {
    HE *he = vertices->at(i)->out;

    // Get total area sum over incident faces
    double total_area = 0;
    do {
      auto f = he->face;
      Vertex v1 = f->edge->vertex->getVertex();
      Vertex v2 = f->edge->next->vertex->getVertex();
      Vertex v3 = f->edge->next->next->vertex->getVertex();

      // face_normal = cross product of (v2 - v1) x (v3 - v1)
      auto face_normal = cross(v2 - v1, v3 - v1);
      // face area = 1/2 | face_normal |
      auto face_area = 0.5 * face_normal.norm();
      total_area += face_area;
      he = he->flip->next;
    } while (he != vertices->at(i)->out);

    if (total_area > 0.0000001) {
      double sum_over_j = 0;
      do {  // iterate over all vertices adjacent to v_i
        int j = he->next->vertex->index;  // get index of adjacent vertex to v_i
        Vertex v_i = he->vertex->getVertex();
        Vertex v_j = he->next->vertex->getVertex();
        Vertex v_b = he->next->next->vertex->getVertex();
        Vertex v_a = he->flip->next->next->vertex->getVertex();

        Vertex a1 = v_i - v_a;
        Vertex a2 = v_j - v_a;
        Vertex b1 = v_j - v_b;
        Vertex b2 = v_i - v_b;

        double cot_alpha = dot(a1, a2) / (cross(a1, a2).norm());
        double cot_beta = dot(b1, b2) / (cross(b1, b2).norm());

        double cot_sum = cot_alpha + cot_beta;
        sum_over_j -= 0.5 * cot_sum / total_area;
        L.insert(i-1, j-1) = 0.5 * cot_sum / total_area;

        he = he ->flip->next;
      } while (he != vertices->at(i)->out);

      L.insert(i-1, i-1) = sum_over_j;
    }
  }

  Eigen::SparseMatrix<double> B(num_vertices, num_vertices);
  B.setIdentity();
  B -= h * L;
  B.makeCompressed();  // optional; tells Eigen to more efficiently store our sparse matrix

  return B;
}

Eigen::VectorXd build_x0(std::vector<HEV*> *vertices) {
  Eigen::VectorXd x0(vertices->size() - 1);
  for (int i = 1; i < vertices->size(); ++i) {
    x0(i-1) = (*vertices)[i]->x;
  }
  return x0;
}

Eigen::VectorXd build_y0(std::vector<HEV*> *vertices) {
  Eigen::VectorXd y0(vertices->size() - 1);
  for (int i = 1; i < vertices->size(); ++i) {
    y0(i-1) = (*vertices)[i]->y;
  }
  return y0;
}

Eigen::VectorXd build_z0(std::vector<HEV*> *vertices) {
  Eigen::VectorXd z0(vertices->size() - 1);
  for (int i = 1; i < vertices->size(); ++i) {
    z0(i-1) = (*vertices)[i]->z;
  }
  return z0;
}

void set_x(std::vector<HEV*> *vertices, const Eigen::VectorXd& x_new) {
  for (int i = 1; i < vertices->size(); ++i) {
    (*vertices)[i]->x = x_new(i-1);
  }
}

void set_y(std::vector<HEV*> *vertices, const Eigen::VectorXd& y_new) {
  for (int i = 1; i < vertices->size(); ++i) {
    (*vertices)[i]->y = y_new(i-1);
  }
}

void set_z(std::vector<HEV*> *vertices, const Eigen::VectorXd& z_new) {
  for (int i = 1; i < vertices->size(); ++i) {
    (*vertices)[i]->z = z_new(i-1);
  }
}

void smooth(std::vector<HEV*> *vertices, double h) {
  // get our matrix representation of B
  Eigen::SparseMatrix<double> F = build_F_operator(vertices, h);

  // initialize Eigen's sparse solver
  Eigen::SparseLU<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int> > solver;

  // the following two lines essentially tailor our solver to our operator F
  solver.analyzePattern(F);
  solver.factorize(F);

  int num_vertices = vertices->size() - 1;

  // initialize our vector representation of rho
  auto x0 = build_x0(vertices);
  auto y0 = build_y0(vertices);
  auto z0 = build_z0(vertices);
  //Eigen::VectorXd rho_vector(num_vertices);
  //for (int i = 1; i < vertices->size(); ++i)
  //  rho_vector(i - 1) = rho(i);  // assuming we can retrieve our given rho values from somewhere

  // have Eigen solve for our phi_vector
  //Eigen::VectorXd phi_vector(num_vertices);
  auto x_new = solver.solve(x0);
  auto y_new = solver.solve(y0);
  auto z_new = solver.solve(z0);
  // phi_vector = solver.solve(rho_vector);

  // do something with phi_vector
  set_x(vertices, x_new);
  set_y(vertices, y_new);
  set_z(vertices, z_new);
}
