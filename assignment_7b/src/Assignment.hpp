#ifndef ASSIGNMENT_HPP
#define ASSIGNMENT_HPP

#include <vector>

#include "PNGMaker.hpp"

class Camera;
class Scene;

using namespace std;

class Assignment {
    public:
        Assignment() = default;

        static void raytrace(Camera camera, Scene scene);
};

#endif