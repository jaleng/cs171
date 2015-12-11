#ifndef SHADER_HPP
#define SHADER_HPP

#include "Utilities.hpp"

#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <vector>

#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

class Shader {
    public:
        static float mode;
        static GLenum program;

        Shader() = default;
        Shader(float mode);
        static void compileShaders();
        static void linkf(float f, char *name);

    private:
        static const char *vert_prog_file_name;
        static const char *frag_prog_file_name;
};

#endif