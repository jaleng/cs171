#include "Assignment.hpp"

#include "UI.hpp"
#include "Scene.hpp"

#define XRES 250
#define YRES 250

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
