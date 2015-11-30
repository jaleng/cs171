#ifndef GEOMETRY_PROCESSING_H_
#define GEOMETRY_PROCESSING_H_
/* 
 * Header file for geometry processing functions
 */

#include <vector>
#include <Eigen/Sparse>

#include "structs.h"
#include "halfedge.h"

Vertex compute_normal(HEV *v);
void reset_normals(std::vector<HEV*> *hevs);

double dot(Vertex v1, Vertex v2);
Vertex cross(Vertex v1, Vertex v2);

void index_vertices(std::vector<HEV*> *vertices);

Eigen::SparseMatrix<double> build_F_operator(std::vector<HEV*> *vertices, double h);

Eigen::VectorXd build_x0(std::vector<HEV*> *vertices);
Eigen::VectorXd build_y0(std::vector<HEV*> *vertices);
Eigen::VectorXd build_z0(std::vector<HEV*> *vertices);

void set_x(std::vector<HEV*> *vertices, const Eigen::VectorXd& x_new);
void set_y(std::vector<HEV*> *vertices, const Eigen::VectorXd& y_new);
void set_z(std::vector<HEV*> *vertices, const Eigen::VectorXd& z_new);

void smooth(std::vector<HEV*> *vertices, double h);


#endif  // GEOMETRY_PROCESSING_H_
