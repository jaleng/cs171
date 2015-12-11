#ifndef UI_HPP
#define UI_HPP

#include "Utilities.hpp"
#include "command_line.hpp"

#include <cmath>
#define _USE_MATH_DEFINES

#include <GL/glut.h>

#include <Eigen/Eigen>

using namespace Eigen;

struct Camera {
    Vec3f position;
    Vec3f axis;
    float angle;

    float near;
    float far;
    float fov;
    float aspect;

    Camera() = default;
    Camera(float *position, float *axis, float angle, float near, float far,
        float fov, float aspect);

    Vector3f getPosition();
    Vector3f getAxis();
    float getAngle();
    float getNear();
    float getFar();
    float getFov();
    float getAspect();
};

class UI {
    public:
        static int xres;
        static int yres;
        static Camera camera;

        static Matrix4f arcball_object_mat;
        static Matrix4f arcball_light_mat;

        static float shader_mode;
        static float scene_scale;
        static bool rebuild_scene;
        static bool wireframe_mode;
        static bool normal_mode;
        static bool io_mode;
        static bool intersect_mode;

        UI() = default;
        UI(int xres, int yres);

        static void handleMouseButton(int button, int state, int x, int y);
        static void handleMouseMotion(int x, int y);
        static void handleKeyPress(unsigned char key, int x, int y);
        static void reshape(int xres, int yres);

    private:
        static int mouse_x;
        static int mouse_y;

        static Matrix4f arcball_rotate_mat;

        static bool arcball_scene;
        static bool mouse_down;

        static Vector3f getArcballVector(int x, int y);
        static void createCamera();
};

#endif