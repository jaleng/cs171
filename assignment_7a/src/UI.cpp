#include "UI.hpp"

// Screen resolution and camera object
int UI::xres = 1000;
int UI::yres = 1000;
Camera UI::camera;

// Per-pixel or per-vertex shading?
float UI::shader_mode = 1.0;
// Scale of the scene
float UI::scene_scale = 1.0;
// Should we rebuild the scene?
bool UI::rebuild_scene = false;
// Is wireframe mode on?
bool UI::wireframe_mode = false;
// Are we drawing the normals in wireframe mode?
bool UI::normal_mode = false;
// Are we drawing the IO test?
bool UI::io_mode = true;
// Are we drawing the intersection of the camera look vector with a primitive?
bool UI::intersect_mode = true;

// Last arcball mouse position
int UI::mouse_x;
int UI::mouse_y;

// Does the arcball rotate the lights as well as the objects?
bool UI::arcball_scene = true;
// Is the left mouse button down?
bool UI::mouse_down = false;

// Arcball rotation matrix for the current update
Matrix4f UI::arcball_rotate_mat = Matrix4f::Identity();
// Overall arcball object rotation matrix
Matrix4f UI::arcball_object_mat = Matrix4f::Identity();
// Overall arcball light rotation matrix
Matrix4f UI::arcball_light_mat = Matrix4f::Identity();

/* Constructs the program's camera. */
Camera::Camera(float *position, float *axis, float angle, float near, float far,
    float fov, float aspect)
{
    for (int i = 0; i < 3; i++) {
        this->position = Vec3f(position);
        this->axis = Vec3f(axis);
    }
    this->angle = angle;
    this->near = near;
    this->far = far;
    this->fov = fov;
    this->aspect = aspect;
}

Vector3f Camera::getPosition() {
    return Vector3f(this->position.x, this->position.y, this->position.z);
}

Vector3f Camera::getAxis() {
    return Vector3f(this->axis.x, this->axis.y, this->axis.z);
}

float Camera::getAngle() {
    return this->angle;
}

float Camera::getNear() {
    return this->near;
}

float Camera::getFar() {
    return this->far;
}

float Camera::getFov() {
    return this->fov;
}

float Camera::getAspect() {
    return this->aspect;
}

/* Initializes the UI with the given resolution, and creates the camera. */
UI::UI(int xres, int yres) {
    UI::xres = xres;
    UI::yres = yres;
    createCamera();
}

/* Sets up the scene's camera. */
void UI::createCamera() {
    float position[3] = {0.0, 0.0, 10.0};
    float axis[3] = {0.0, 0.0, 1.0};
    float angle = 0.0;
    float near = 0.1;
    float far = 500.0;
    float fov = 60.0;
    float aspect = (float) xres / yres;

    // Create the Camera struct
    camera = Camera(position, axis, angle, near, far, fov, aspect);
}

/* Gets an arcball vector from a position in the program's window. */
Vector3f UI::getArcballVector(int x, int y) {
    // Convert from screen space to NDC
    Vector3f p(2.0 * x / xres - 1.0, -(2.0 * y / yres - 1.0), 0.0);
    // Compute the appropriate z-coordinate, and make sure it's normalized
    float squared_length = p.squaredNorm();
    if (squared_length < 1.0)
        p[2] = -sqrtf(1.0 - squared_length);
    else
        p /= sqrtf(squared_length);

    return p;
}

/* Handles mouse click events. */
void UI::handleMouseButton(int button, int state, int x, int y) {
    // If the action pertains to the left button...
    if (button == GLUT_LEFT_BUTTON) {
        // If the button is being held down, update the start of the arcball
        // rotation, and store that the button is currently down
        mouse_down = state == GLUT_DOWN;
        if (state == GLUT_DOWN) {
            mouse_x = x;
            mouse_y = y;
            mouse_down = true;
        }
        // Otherwise, store that the button is up
        else
            mouse_down = false;
    }
}

/* Handles mouse motion events. */
void UI::handleMouseMotion(int x, int y) {
    // If the left button is being clicked, and the mouse has moved, and the
    // mouse is in the window, then update the arcball UI
    if (mouse_down && (x != mouse_x || y != mouse_y) &&
        (x >= 0 && x < xres && y >= 0 && y < yres))
    {
        // Set up some matrices we need
        Matrix4f camera_to_ndc = Matrix4f();
        Matrix4f world_to_camera = Matrix4f();
        glGetFloatv(GL_PROJECTION_MATRIX, camera_to_ndc.data());
        glGetFloatv(GL_MODELVIEW_MATRIX, world_to_camera.data());
        Matrix3f ndc_to_world = camera_to_ndc.topLeftCorner(3, 3).inverse();

        // Get the two arcball vectors by transforming from NDC to camera
        // coordinates, ignoring translation components
        Vector3f va =
            (ndc_to_world * getArcballVector(mouse_x, mouse_y)).normalized();
        Vector3f vb = (ndc_to_world * getArcballVector(x, y)).normalized();
        
        // Compute the angle between them and the axis to rotate around
        // (this time rotated into world space, where the matrix is applied)
        Vector3f arcball_axis =
            (world_to_camera.topLeftCorner(3, 3).transpose() * va.cross(vb)).normalized();
        float arcball_angle = acos(fmax(fmin(va.dot(vb), 1.0), -1.0));

        // Update current arcball rotation and overall object rotation matrices
        makeRotateMat(arcball_rotate_mat.data(), arcball_axis[0],
            arcball_axis[1], arcball_axis[2], arcball_angle);
        arcball_object_mat = arcball_rotate_mat * arcball_object_mat;
        
        // If the arcball should rotate the entire scene, update the light
        // rotation matrix too 
        if (arcball_scene)
            arcball_light_mat = arcball_rotate_mat * arcball_light_mat;
        
        // Update the arcball start position
        mouse_x = x;
        mouse_y = y;
        
        // Update the image
        glutPostRedisplay();
    }
}

/* Handles keyboard events. */
void UI::handleKeyPress(unsigned char key, int x, int y) {
    // Q quits the program
    if (key == 'q' || key == 27) {
        CommandLine::run();
    }
    // B toggles whether the arcball rotates the lights along with the objects
    else if (key == 'b') {
        arcball_scene = !arcball_scene;
    }
    // M toggles between per-pixel and per-vertex shading
    else if (key == 'm') {
        shader_mode = !((int) shader_mode);
    }
    // W blows up the scene
    else if (key == 'w') {
        scene_scale *= 1.1;
    }
    // S shrinks the scene
    else if (key == 's') {
        scene_scale /= 1.1;
    }
    // R forces a refresh of the scene's entities
    else if (key == 'r') {
        rebuild_scene = true;
    }
    // T toggles wireframe mode
    else if (key == 't') {
        wireframe_mode = !wireframe_mode;
    }
    // N toggles normal mode
    else if (key == 'n') {
        normal_mode = !normal_mode;
    }
    // O toggles IO mode
    else if (key == 'o') {
        io_mode = !io_mode;
    }
    // I toggles intersect mode
    else if (key == 'i') {
        intersect_mode = !intersect_mode;
    }
    glutPostRedisplay();
}

/* Handles window resize events. */
void UI::reshape(int xres, int yres) {
    // Update the internal resolution
    UI::xres = xres;
    UI::yres = yres;
}
