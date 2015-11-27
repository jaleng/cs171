#include <iostream>
#include <string>
#include <fstream>

#include <GL/glew.h>
#include <GL/glut.h>
#include <math.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "SceneParser.h"
#include "PointLight.h"
#include "Object.h"
#include "Quaternion.h"
#include "Arcball.h"

#include "structs.h"
#include "halfedge.h"

#define _USE_MATH_DEFINES

/** Point lights to be used in the scene **/
std::vector<PointLight> lights;

/** Objects in the scene **/
std::vector<Object> objects;

//// Forward declarations
void init(void);
void reshape(int width, int height);
void display(void);
void init_lights();
void set_lights();
void draw_objects();
void mouse_pressed(int button, int state, int x, int y);
void mouse_moved(int x, int y);
void key_pressed(unsigned char key, int x, int y);
double deg2rad(double angle);
double rad2deg(double angle);

//// Camera Parameters
double cam_position[3];
double cam_orientation_axis[3];
/** Angle in degrees **/
double cam_orientation_angle;
/** Frustrum parameters **/
double near_param, far_param,
       left_param, right_param,
       top_param,  bottom_param;

/** Cursor positions used for arcball calculations **/
int px_start, py_start;
int px_current, py_current;
int window_width, window_height;

/** Quaternions used to calculate arcball rotations **/
Quaternion last_rotation;
Quaternion current_rotation;

/** Global parameters for user input memory **/
bool is_pressed = false;
bool wireframe_mode = false;

/** Initialization of gl, lights, arcball rotation **/
void init() {
  // Use Gouraud shading
  glShadeModel(GL_SMOOTH);

  // Do backface culling
  glEnable(GL_CULL_FACE);
  glCullFace(GL_BACK);

  // Do depth buffering
  glEnable(GL_DEPTH_TEST);

  // Automatically normalize normal vectors
  glEnable(GL_NORMALIZE);

  // Enable vertex array and normal array functionality
  glEnableClientState(GL_VERTEX_ARRAY);
  glEnableClientState(GL_NORMAL_ARRAY);

  // Modify projection matrix
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  // Create perspective projection matrix using the frustrum parameters
  glFrustum(left_param, right_param,
            bottom_param, top_param,
            near_param, far_param);

  // Go back to model view matrix
  glMatrixMode(GL_MODELVIEW);

  init_lights();

  // Set up rotation quaternions
  last_rotation = Quaternion::identity();
  current_rotation = Quaternion::identity();
}

/** Called when gl window is resized **/
void reshape(int width, int height) {
  // Prevent width, height from being 0 to prevent DIVIDE_BY_ZERO
  height = (height == 0) ? 1 : height;
  width = (width == 0) ? 1 : width;

  window_width = width;
  window_height = height;
  glViewport(0, 0, width, height);
  glutPostRedisplay();
}

/** Display objects on gl window **/
void display(void) {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
  glLoadIdentity();

  // Apply inverse camera transform (world-space -> camera space)
  glRotated(-cam_orientation_angle,
            cam_orientation_axis[0], cam_orientation_axis[1], cam_orientation_axis[2]);
  glTranslated(-cam_position[0], -cam_position[1], -cam_position[2]);

  // Arcball rotation
  auto arcball_matrix = (current_rotation * last_rotation).getRotationMatrix();
  glMultMatrixd(&arcball_matrix[0]);

  set_lights();

  draw_objects();

  // For double buffering
  glutSwapBuffers();
}

/** Initialize GL lights **/
void init_lights() {
  glEnable(GL_LIGHTING);

  int num_lights = lights.size();

  for (int i = 0; i < num_lights; ++i) {
    int light_id = GL_LIGHT0 + i;
    glEnable(light_id);

    std::array<float, 3> color;
    for (int j = 0; j < 3; ++j) {
      color[j] = static_cast<float>(lights[i].color[j]);
    }

    glLightfv(light_id, GL_AMBIENT, &color[0]);
    glLightfv(light_id, GL_DIFFUSE, &color[0]);
    glLightfv(light_id, GL_SPECULAR, &color[0]);

    // Set attenuation
    glLightf(light_id, GL_QUADRATIC_ATTENUATION, static_cast<float>(lights[i].attenuation_k));
  }
}

/** Set light positions **/
void set_lights() {
  int num_lights = lights.size();

  for (int i = 0; i < num_lights; ++i) {
    int light_id = GL_LIGHT0 + i;
    std::array<float, 4> position;
    for (int j = 0; j < 4; ++j) {
      position[j] = lights[i].position[j];
    }
    glLightfv(light_id, GL_POSITION, &position[0]);
  }
}

/** Draw objects **/
void draw_objects() {
  int num_objects = objects.size();

  for (int i = 0; i < num_objects; ++i) {
    // Push a matrix so we can modify it for this object only
    glPushMatrix();
    {
      // Apply object transformations (in reverse order because post-multiplication)
      for (auto it = objects[i].transforms.crbegin();
           it != objects[i].transforms.crend(); ++it) {
        switch (it->transform_type) {
        case TransformType::TRANSLATION:
          glTranslated((*it).translation[0],
                       (*it).translation[1],
                       (*it).translation[2]);
          break;
        case TransformType::ROTATION:
          glRotated(rad2deg((*it).rotation_angle),
                    (*it).rotation[0],
                    (*it).rotation[1],
                    (*it).rotation[2]);
          break;
        case TransformType::SCALING:
          glScaled((*it).scaling[0],
                   (*it).scaling[1],
                   (*it).scaling[2]);
          break;
        }
      }

      // Set material lighting properties
      std::array<float, 3> ambient_reflect;
      std::array<float, 3> diffuse_reflect;
      std::array<float, 3> specular_reflect;

      for (int j = 0; j < 3; ++j) {
        ambient_reflect[j] = static_cast<float>(objects[i].ambient_reflect[j]);
        diffuse_reflect[j] = static_cast<float>(objects[i].diffuse_reflect[j]);
        specular_reflect[j] = static_cast<float>(objects[i].specular_reflect[j]);
      }
      glMaterialfv(GL_FRONT, GL_AMBIENT, &ambient_reflect[0]);
      glMaterialfv(GL_FRONT, GL_DIFFUSE, &diffuse_reflect[0]);
      glMaterialfv(GL_FRONT, GL_SPECULAR, &specular_reflect[0]);
      glMaterialf(GL_FRONT, GL_SHININESS, static_cast<float>(objects[i].shininess));


      glVertexPointer(3, GL_DOUBLE, 0, &objects[i].vertex_buffer[0]);
      glNormalPointer(GL_DOUBLE, 0, &objects[i].normal_buffer[0]);

      int buffer_size = objects[i].vertex_buffer.size();

      if (!wireframe_mode)
        glDrawArrays(GL_TRIANGLES, 0, buffer_size);
      else
        for (int j = 0; j < buffer_size; j += 3)
          glDrawArrays(GL_LINE_LOOP, j, 3);
    }

    glPopMatrix();
  }
}

/** Called when mouse is pressed **/
void mouse_pressed(int button, int state, int x, int y) {
  // If the left-mouse button was clicked down, then...
  if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
    // Set arcball start coords
    px_start = x;
    py_start = y;

    /* Since the mouse is being pressed down, we set our 'is_pressed"
     * boolean indicator to true.
     */
    is_pressed = true;
  }
  // If the left-mouse button was released up, then...
  else if (button == GLUT_LEFT_BUTTON && state == GLUT_UP) {
    // Reset arcball rotation to the current one
    last_rotation = current_rotation * last_rotation;
    current_rotation = Quaternion::identity();

    // Mouse is no longer being pressed, so set our indicator to false.
    is_pressed = false;
  }
}

/** Called when mouse is moved **/
void mouse_moved(int x, int y) {
  // If the left-mouse button is being clicked down...
  if (is_pressed) {
    // Set current arcball coords
    px_current = x;
    py_current = y;

    // Calculate the current rotation
    current_rotation =
      ArcBall::compute_rotation_quaternion(px_current, py_current,
                                           px_start, py_start,
                                           window_width, window_height);

    // Rerender
    glutPostRedisplay();
  }
}

/** Called when key is pressed **/
void key_pressed(unsigned char key, int x, int y) {
  // If 'q' is pressed, quit the program.
  if (key == 'q') {
    exit(0);
  }
  /* If 't' is pressed, toggle our 'wireframe_mode' boolean to make OpenGL
   * render our cubes as surfaces of wireframes.
   */
  else if (key == 't') {
    wireframe_mode = !wireframe_mode;
    /* Tell OpenGL that it needs to re-render our scene with the cubes
     * now as wireframes (or surfaces if they were wireframes before).
     */
    glutPostRedisplay();
  }
}

/** Convert angle values from degrees to radians **/
double deg2rad(double angle) {
  return angle * M_PI / 180.0;
}

/** Convert angle values from radians to degrees **/
double rad2deg(double angle) {
  return angle * 180.0 / M_PI;
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
float dot(Vertex v1, Vertex v2) {
  return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

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

/** function to assign each vertex in our mesh to an index **/
void index_vertices(std::vector<HEV*> *vertices) {
  for (int i = 1; i < vertices->size(); ++i)  // start at 1 because obj files are 1-indexed
    vertices->at(i)->index = i;  // assign each vertex an index
}

/** Make discrete Laplacian operator **/
// function to construct our B operator in matrix form
Eigen::SparseMatrix<double> build_F_operator(std::vector<HEV*> *vertices, float h) {  
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
    float total_area = 0;
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

    float sum_over_j = 0;
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

      float cot_alpha = dot(a1, a2) / (cross(a1, a2).norm());
      float cot_beta = dot(b1, b2) / (cross(b1, b2).norm());

      float cot_sum = cot_alpha + cot_beta;
      sum_over_j -= 0.5 * cot_sum / total_area;
      L.insert(i-1, j-1) = 0.5 * cot_sum / total_area;

      he = he ->flip->next;
    } while (he != vertices->at(i)->out);

    L.insert(i-1, i-1) = sum_over_j;
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

void smooth(std::vector<HEV*> *vertices, float h) {
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


/** Parse and display a scene with the given window resolution **/
int main(int argc, char *argv[]) {
  // Get cli args:
  // ./opengl_renderer [scene_description_file.txt] [xres] [yres] [mode]
  std::ifstream scene_desc_file_stream{argv[1]};
  int xres = atoi(argv[2]);
  int yres = atoi(argv[3]);

  // Set global window params for arcball calculation
  window_width = xres;
  window_height = yres;

  // Parse the scene description file
  auto scene = SceneParser::parse_scene(scene_desc_file_stream);



  ///////////////////////////////////////////////////////////////////////
  //////////// MAKE HALF EDGES //////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////
  /** Make half edges **/

  auto mesh_data = std::make_unique<Mesh_Data>();
  // TODO: get the ObjectData
  auto obj_data = scene->objid_to_data_up->begin()->second;
  // TODO: make std::vector<Vertex*> *vertices for mesh
  std::vector<Vertex> vertices_for_HE_build;
  for (const auto& triple : obj_data.vertices) {
    vertices_for_HE_build.emplace_back(static_cast<float>(triple.x),
                                       static_cast<float>(triple.y),
                                       static_cast<float>(triple.z));
  }
  std::vector<Vertex*> pvertices_for_HE_build;
  // Prep for 1-indexing
  pvertices_for_HE_build.push_back(nullptr);
  for (auto& v : vertices_for_HE_build) {
    pvertices_for_HE_build.push_back(&v);
  }

  // TODO: make std::vector<FaceforHE*> *faces
  std::vector<FaceforHE> faces_for_HE_build;
  for (const auto& f : obj_data.faces) {
    faces_for_HE_build.emplace_back(f.v1_idx, f.v2_idx, f.v3_idx);
  }
  std::vector<FaceforHE*> pfaces_for_HE_build;
  for (auto& f : faces_for_HE_build) {
    pfaces_for_HE_build.push_back(&f);
  }

  // TODO: build half edges
  mesh_data->vertices = &pvertices_for_HE_build;
  mesh_data->faces = &pfaces_for_HE_build;
  auto hevs = new std::vector<HEV*>();
  index_vertices(hevs);
  auto hefs = new std::vector<HEF*>();
  build_HE(mesh_data.get(), hevs, hefs);

  //////////////////////////////////////////////////////////////////////
  ////////////// SMOOTH ////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////
  smooth(hevs, .0001);

  //////////////////////////////////////////////////////////////////////
  //////////// COMPUTE NORMALS  ////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////
  // TODO(jg): compute normals
  for (auto& hev : *hevs) {
    if (hev == nullptr) {
      continue;
    }
    auto computed_normal = compute_normal(hev);
    hev->normal.x = computed_normal.x;
    hev->normal.y = computed_normal.y;
    hev->normal.z = computed_normal.z;
  }

  //////////////////////////////////////////////////////////////////////
  /////////// Put results in objects ///////////////////////////////////
  //////////////////////////////////////////////////////////////////////
  objects = *(scene->objects_up);
  objects[0].vertex_buffer.clear();
  objects[0].normal_buffer.clear();

  for (const auto& f : *hefs) {
    auto v1 = f->edge->vertex;
    auto v2 = f->edge->next->vertex;
    auto v3 = f->edge->next->next->vertex;
    objects[0].vertex_buffer.emplace_back(v1->x, v1->y, v1->z);
    objects[0].vertex_buffer.emplace_back(v2->x, v2->y, v2->z);
    objects[0].vertex_buffer.emplace_back(v3->x, v3->y, v3->z);

    objects[0].normal_buffer.emplace_back(v1->normal.x, v1->normal.y, v1->normal.z);
    objects[0].normal_buffer.emplace_back(v2->normal.x, v2->normal.y, v2->normal.z);
    objects[0].normal_buffer.emplace_back(v3->normal.x, v3->normal.y, v3->normal.z);
  }

  lights = *(scene->lights_up);

  // Set global camera parameters
  for (int i = 0; i < 3; ++i) {
    cam_position[i] = scene->camera_up->position[i];
    cam_orientation_axis[i] = scene->camera_up->orientation_axis[i];
  }
  cam_orientation_angle = rad2deg(scene->camera_up->orientation_angle);
  near_param   = scene->camera_up->near;
  far_param    = scene->camera_up->far;
  left_param   = scene->camera_up->left;
  right_param  = scene->camera_up->right;
  top_param    = scene->camera_up->top;
  bottom_param = scene->camera_up->bottom;

  // Do GL stuff
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
  glutInitWindowSize(xres, yres);
  glutInitWindowPosition(0, 0);
  using std::string;
  string window_title = string("opengl_renderer - ") + string(argv[1]);
  glutCreateWindow(window_title.c_str());

  init();
  glutDisplayFunc(display);
  glutReshapeFunc(reshape);
  glutMouseFunc(mouse_pressed);
  glutMotionFunc(mouse_moved);
  glutKeyboardFunc(key_pressed);
  glutMainLoop();
}
