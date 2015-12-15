#include <vector>
#include <memory>
#include <algorithm>
#include <limits>
#include <iostream>
#include <string>

#include "Assignment.hpp"

#include "UI.hpp"
#include "Scene.hpp"

#define XRES 250
#define YRES 250

using std::unique_ptr;

unique_ptr<vector<PAT>> getPATs(const Renderable& ren);
unique_ptr<vector<PAT>> getPATs(const Scene& scene);

Vector3d getB(const Camera& camera);
Vector3d getA(const Camera& camera, int i, int j);

lighting(Vector3d lit_pos, Vector3d normal,
         const Primitive& prm, const vector<PointLight>& lights,
         Vector3d cam_pos);
bool isShaded(const PointLight& light, Vector3d lit_pos, const Primitive& prm);

/* Ray traces the scene. */
void Assignment::raytrace(Camera camera, Scene scene) {
    // LEAVE THIS UNLESS YOU WANT TO WRITE YOUR OWN OUTPUT FUNCTION
    PNGMaker png = PNGMaker(XRES, YRES);

    // REPLACE THIS WITH YOUR CODE
    for (int i = 0; i < XRES; i++) {
        for (int j = 0; j < YRES; j++) {
            png.setPixel(i, j, 1.0, 1.0, 1.0);
        }
    }

    // LEAVE THIS UNLESS YOU WANT TO WRITE YOUR OWN OUTPUT FUNCTION
    if (png.saveImage())
        printf("Error: couldn't save PNG image\n");
}
