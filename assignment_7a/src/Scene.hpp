#ifndef SCENE_HPP
#define SCENE_HPP

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cstdio>
#include <vector>

#include <Eigen/Eigen>

#include "command_line.hpp"
#include "model.hpp"
#include "Utilities.hpp"

using namespace std;
using namespace Eigen;

struct PointLight {
    float position[4];
    float color[3];
    float k;

    PointLight() = default;
    PointLight(float *position, float *color, float k);
};

// struct Primitive {
//     float e;
//     float n;
//     Matrix4f rotate;
//     Vec3f scale;
//     Vec3f translate;

//     int ures = 100;
//     int vres = 50;

//     Primitive() = default;
//     Primitive(float e, float n, float *scale, float *rotate, float theta,
//         float *translate);
//     Primitive(float e, float n, float *scale, float *rotate, float theta,
//         float *translate, int ures, int vres);

//     Vec3f getVertex(float u, float v);
//     Vec3f getNormal(Vec3f *vertex);
// };

// struct Object {
//     vector<Vec3f> vertices;
//     vector<Vec3f> normals;
//     vector<Vec3i> faces;

//     Object();
// };

class Scene {
    public:
        static vector<Object*> root_objs;

        static unordered_map<Primitive*, unsigned int> prm_tessellation_start;
        // static vector<Object> objects;
        static vector<PointLight> lights;

        static vector<Vector3f> vertices;
        static vector<Vector3f> normals;

        Scene();
        static void createLights();
        static int getLightCount();

        static void update();

    private:
        static bool needs_update;

        static void generateVertex(Primitive *prm, float u, float v);
        static void tessellatePrimitive(Primitive *prm);
        static void tessellateObject(Object *obj);
        // static void setupObject(Object *object);
};

#endif