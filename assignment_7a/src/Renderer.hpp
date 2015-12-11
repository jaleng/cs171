#ifndef RENDERER_HPP
#define RENDERER_HPP

#include "Utilities.hpp"

#include "Scene.hpp"
#include "Shader.hpp"
#include "UI.hpp"
#include "Assignment.hpp"

#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

class Renderer {
    public:
        Renderer(int x, int y);

        static void init();
        static void start();
        
        // static void addPrimitive(float e, float n, float *scale, float *rotate,
        //     float theta, float *translate);
        // static void addObject(char *file_name);
        static void updateScene();

    private:
        static Scene scene;
        static Shader shader;
        static UI ui;
        static GLuint display_list;
        static GLuint vb_array;
        static GLuint vb_objects[2];

        static void initLights();
        static void setupLights();

        static void display();
        static void reshape(int xres, int yres);

        static void checkUIState();
        // static uint drawObjects(uint start);
        static void draw(Renderable* ren, int depth);
        static void drawPrimitive(Primitive* prm);
        static void drawObject(Object* obj, int depth);
        static void drawAxes();
        static void transform(const Transformation& trans);
};

#endif