#include <iostream>
#include <string>
#include <fstream>

#include <GL/glew.h>
#include <GL/glut.h>
#include <math.h>
#include <memory>

#include "PointLight.h"
#include "Object.h"
#include "Quaternion.h"
#include "Arcball.h"
#include "animation.h"

#define _USE_MATH_DEFINES

/** Point lights to be used in the scene **/
std::vector<PointLight> lights;

/** Objects in the scene **/
std::vector<Object> objects;

/** I-bar **/
GLUquadricObj *quadratic;

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
void drawIBar();
void show_frame(int frame);
void show_next_frame();

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

/** Animation vars **/
int current_frame = 0;
Animation *animation;

/** Initialization of gl, lights, arcball rotation **/
void init() {
  // Initialize quadratic object for I-bar usage
  quadratic = gluNewQuadric();

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

  //init_lights();

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

  //set_lights();

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

      drawIBar();
      /*
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
      */
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
  } else if (key == 'f') {
    show_next_frame();
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

/** Draw I-Bar **/
void drawIBar() {
  /* Parameters for drawing the cylinders */
  float cyRad = 0.2, cyHeight = 1.0;
  int quadStacks = 4, quadSlices = 4;

  glPushMatrix();
  glColor3f(0, 0, 1);
  glTranslatef(0, cyHeight, 0);
  glRotatef(90, 1, 0, 0);
  gluCylinder(quadratic, cyRad, cyRad, 2.0 * cyHeight, quadSlices, quadStacks);
  glPopMatrix();

  glPushMatrix();
  glColor3f(0, 1, 1);
  glTranslatef(0, cyHeight, 0);
  glRotatef(90, 0, 1, 0);
  gluCylinder(quadratic, cyRad, cyRad, cyHeight, quadSlices, quadStacks);
  glPopMatrix();

  glPushMatrix();
  glColor3f(1, 0, 1);
  glTranslatef(0, cyHeight, 0);
  glRotatef(-90, 0, 1, 0);
  gluCylinder(quadratic, cyRad, cyRad, cyHeight, quadSlices, quadStacks);
  glPopMatrix();

  glPushMatrix();
  glColor3f(1, 1, 0);
  glTranslatef(0, -cyHeight, 0);
  glRotatef(-90, 0, 1, 0);
  gluCylinder(quadratic, cyRad, cyRad, cyHeight, quadSlices, quadStacks);
  glPopMatrix();

  glPushMatrix();
  glColor3f(0, 1, 0);
  glTranslatef(0, -cyHeight, 0);
  glRotatef(90, 0, 1, 0);
  gluCylinder(quadratic, cyRad, cyRad, cyHeight, quadSlices, quadStacks);
  glPopMatrix();
}

void show_frame(int frame) {
  objects[0].transforms.clear();
  objects[0].transforms.push_back(animation->ft[frame].scale);
  objects[0].transforms.push_back(animation->ft[frame].rotation);
  objects[0].transforms.push_back(animation->ft[frame].translation);
}

void show_next_frame() {
  if (current_frame < animation->number_frames - 1) {
    ++current_frame;
  } else {
    current_frame = 0;
  }
  show_frame(current_frame);
}

std::unique_ptr<Animation> parse_animation(std::ifstream& file_stream) {
  using std::string;
  using std::stringstream;

  string line;
  getline(file_stream, line);
  int number_frames = 0;
  stringstream line_stream{line};
  line_stream >> number_frames;

  auto animation = std::make_unique<Animation>(number_frames);

  while (!file_stream.eof()) {
    string line;
    getline(file_stream, line);
    if (line.empty()) {
      break;
    }
    int number_frames = 0;
    stringstream line_stream{line};
    string token;
    line_stream >> token;
    assert(token == "Frame");
    int frame_number;
    double x, y, z, theta;
    line_stream >> frame_number;
    animation->keyframes.push_back(frame_number);

    getline(file_stream, line);
    line_stream.str("");
    line_stream.clear();
    line_stream << line;
    line_stream >> token >> x >> y >> z;
    assert(token == "translation");
    animation->ft[frame_number].translation.set(x, y, z);

    getline(file_stream, line);
    line_stream.str("");
    line_stream.clear();
    line_stream << line;
    line_stream >> token >> x >> y >> z;
    assert(token == "scale");
    animation->ft[frame_number].scale.set(x, y, z);

    getline(file_stream, line);
    line_stream.str("");
    line_stream.clear();
    line_stream << line;
    line_stream >> token >> x >> y >> z >> theta;
    assert(token == "rotation");
    animation->ft[frame_number].rotation.set(x, y, z, deg2rad(theta));
  }
  return std::move(animation);
}

/** Parse and display a scene with the given window resolution **/
int main(int argc, char *argv[]) {
  // Get cli args:
  // ./opengl_renderer [scene_description_file.txt] [xres] [yres]
  std::ifstream animation_desc_file_stream{argv[1]};
  int xres = atoi(argv[2]);
  int yres = atoi(argv[3]);

  // Set global window params for arcball calculation
  window_width = xres;
  window_height = yres;

  // Parse animation
  initialize_catmull_rom_B();
  auto animation_up = parse_animation(animation_desc_file_stream);
  animation = animation_up.get();
  animation->interpolate();

  // Set global camera parameters
  // Camera hardcoded
  {
    // Position
    cam_position[0] = 0;
    cam_position[1] = 0;
    cam_position[2] = 40;
    // Angle
    cam_orientation_angle = 0;
    // Frustrum parameters
    near_param   = 1.0;
    far_param    = 60.0;
    left_param   = -1.0;
    right_param  = 1.0;
    bottom_param = -1.0;
    top_param    = 1.0;
  }

  // Put an object in the object vector to represent the I-beam
  objects.emplace_back();
  show_frame(0);

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
