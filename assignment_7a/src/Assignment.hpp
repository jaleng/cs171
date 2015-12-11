#ifndef ASSIGNMENT_HPP
#define ASSIGNMENT_HPP

class Camera;

class Assignment {
    public:
        Assignment() = default;

        static void drawIOTest();
        static void drawIntersectTest(Camera *camera);
};

#endif