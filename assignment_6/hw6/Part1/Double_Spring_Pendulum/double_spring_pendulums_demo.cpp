/* CS/CNS 171
 * Fall 2015
 * Written by Kevin (Kevli) Li (Class of 2016)
 *
 * This program is meant to simulate a 2D double spring pendulum using the
 * discrete Lagrangian.
 */

#include <GL/glew.h>
#include <GL/glut.h>

#include <cmath>
#include <cfloat>
#include <iostream>
#include <sstream>
#include <vector>

using namespace std;

struct Point
{
    float x;
    float y;
};

struct Color
{
    float r;
    float g;
    float b;
};

struct Spring_Pendulum
{
    float m;
    
    float x;
    float y;
    
    float px;
    float py;
    
    float k;
    float rl;
};

const float cam_position[] = {0, 0, 2};

const float near_param = 1, far_param = 4,
            left_param = -8, right_param = 8,
            top_param = 6, bottom_param = -10;

const float light_color[3] = {1, 1, 1};
const float light_position[3] = {0, 0, -2};

const float ambient_reflect[3] = {0.3, 0.2, 0.4};
const float diffuse_reflect[3] = {0.7, 0.2, 0.8};
const float specular_reflect[3] = {1, 1, 1};
const float shininess = 0.1;

const float dt = 0.01;
const float g = -9.8;
float t = 0;

const int max_num_points = 1000;

float ke, pe;
float min_total = FLT_MAX, max_total = -FLT_MAX;

float lagrangian_0;

Spring_Pendulum m1;
Spring_Pendulum m2;

vector<Point> path1;
vector<Point> path2;
vector<Color> path_colors1;
vector<Color> path_colors2;

void trace_path();
void draw_spring_pendulums();
void draw_text();

void init(void)
{
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);

    glPointSize(2);

    glEnable(GL_LIGHT0);

    glLightfv(GL_LIGHT0, GL_AMBIENT, light_color);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_color);
    glLightfv(GL_LIGHT0, GL_SPECULAR, light_color);
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);

    glMaterialfv(GL_FRONT, GL_AMBIENT, ambient_reflect);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuse_reflect);
    glMaterialfv(GL_FRONT, GL_SPECULAR, specular_reflect);
    glMaterialf(GL_FRONT, GL_SHININESS, shininess);
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    
    glOrtho(left_param, right_param,
            bottom_param, top_param,
            near_param, far_param);
    
    glMatrixMode(GL_MODELVIEW);
}

void reshape(int width, int height)
{
    height = (height == 0) ? 1 : height;
    width = (width == 0) ? 1 : width;
    
    glViewport(0, 0, width, height);
    
    glutPostRedisplay();
}

void display(void)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    glTranslatef(-cam_position[0], -cam_position[1], -cam_position[2]);
    
    trace_path();
    
    glEnable(GL_LIGHTING);
    draw_spring_pendulums();
    glDisable(GL_LIGHTING);

    draw_text();
    
    glutSwapBuffers();
}

float compute_lagrangian()
{
    ke = 1.0 / 2.0 * m1.m * ((m1.px / m1.m) * (m1.px / m1.m) + (m1.py / m1.m) * (m1.py / m1.m))
         + 1.0 / 2.0 * m2.m * ((m2.px / m2.m) * (m2.px / m2.m) + (m2.py / m2.m) * (m2.py / m2.m));
    pe = 1.0 / 2.0 * m1.k * (sqrt(m1.x * m1.x + m1.y * m1.y) - m1.rl) *
                                  (sqrt(m1.x * m1.x + m1.y * m1.y) - m1.rl)
         + 1.0 / 2.0 * m2.k * (sqrt((m1.x - m2.x) * (m1.x - m2.x) + (m1.y - m2.y) * (m1.y - m2.y)) - m2.rl) *
                                  (sqrt((m1.x - m2.x) * (m1.x - m2.x) + (m1.y - m2.y) * (m1.y - m2.y)) - m2.rl)
         - m1.m * g * m1.y
         - m2.m * g * m2.y;

    float total = ke + pe;
    min_total = (total < min_total) ? total : min_total;
    max_total = (total > max_total) ? total : max_total;
    
    return ke - pe;
}

void update_path()
{    
    if(path1.size() == max_num_points)
    {
        path1.erase(path1.begin());
        path2.erase(path2.begin());
        
        path_colors1.erase(path_colors1.begin());
        path_colors2.erase(path_colors2.begin());
    }
    
    Point point1;
    point1.x = m1.x;
    point1.y = m1.y;
    
    Point point2;
    point2.x = m2.x;
    point2.y = m2.y;
    
    float lagrangian_norm = abs(compute_lagrangian() / lagrangian_0);
    lagrangian_norm = (lagrangian_norm > 1.0) ? 1.0 : lagrangian_norm;
    
    Color color1;
    color1.r = lagrangian_norm;
    color1.g = lagrangian_norm;
    color1.b = 1.0 - lagrangian_norm;
    
    Color color2;
    color2.r = 1.0 - lagrangian_norm;
    color2.g = lagrangian_norm;
    color2.b = lagrangian_norm;
    
    path1.push_back(point1);
    path_colors1.push_back(color1);
    
    path2.push_back(point2);
    path_colors2.push_back(color2);
}

void update_pendulums()
{
    //// update m1.x, m1.y, m1.px, m1.py,
    ////        m2.x, m2.y, m2.px, m2.py

    auto x1s = m1.x;
    auto y1s = m1.y;
    auto x2s = m2.x;
    auto y2s = m2.y;
    auto px1s = m1.px;
    auto py1s = m1.py;
    auto px2s = m2.px;
    auto py2s = m2.py;
    auto l1 = m1.rl;
    auto l2 = m2.rl;
    auto k1 = m1.k;
    auto k2 = m2.k;

    // Sqrt[x1s^2+y1s^2]
    auto sqrtx1y1 = sqrt(x1s*x1s+y1s*y1s);
    // Sqrt[(-x1s+x2s)^2+(-y1s+y2s)^2]
    auto sqrtx2mx1y2my1 = sqrt((x2s-x1s)*(x2s-x1s)+(y2s-y1s)*(y2s-y1s));

    // formulas pasted from Mathematica and altered for syntax and use
    // of the above calculated values
    auto px1_next = -((-dt*k2*l2*x1s*sqrtx1y1+dt*k2*l2*x2s*sqrtx1y1-dt*k1*l1*x1s*sqrtx2mx1y2my1-px1s*sqrtx1y1*sqrtx2mx1y2my1+dt*k1*x1s*sqrtx1y1*sqrtx2mx1y2my1+dt*k2*x1s*sqrtx1y1*sqrtx2mx1y2my1-dt*k2*x2s*sqrtx1y1*sqrtx2mx1y2my1)/(sqrtx1y1*sqrtx2mx1y2my1));
    auto px2_next = px2s-(dt*k2*(-x1s+x2s)*(-l2+sqrtx2mx1y2my1))/sqrtx2mx1y2my1;
    auto py1_next = dt*g*m1.m+py1s-(dt*k1*y1s*(-l1+sqrtx1y1))/sqrtx1y1+(dt*k2*(-y1s+y2s)*(-l2+sqrtx2mx1y2my1))/sqrtx2mx1y2my1;
    auto py2_next = dt*g*m2.m+py2s-(dt*k2*(-y1s+y2s)*(-l2+sqrtx2mx1y2my1))/sqrtx2mx1y2my1;
    auto x1_next = (dt*(px1s+(m1.m*x1s)/dt-(dt*k1*x1s*(-l1+sqrtx1y1))/sqrtx1y1+(dt*k2*(-x1s+x2s)*(-l2+sqrtx2mx1y2my1))/sqrtx2mx1y2my1))/m1.m;
    auto x2_next = (dt*(px2s+(m2.m*x2s)/dt-(dt*k2*(-x1s+x2s)*(-l2+sqrtx2mx1y2my1))/sqrtx2mx1y2my1))/m2.m;
    auto y1_next = (dt*(dt*g*m1.m+py1s+(m1.m*y1s)/dt-(dt*k1*y1s*(-l1+sqrtx1y1))/sqrtx1y1+(dt*k2*(-y1s+y2s)*(-l2+sqrtx2mx1y2my1))/sqrtx2mx1y2my1))/m1.m;
    auto y2_next = (dt*(dt*g*m2.m+py2s+(m2.m*y2s)/dt-(dt*k2*(-y1s+y2s)*(-l2+sqrtx2mx1y2my1))/sqrtx2mx1y2my1))/m2.m;

    m1.px = px1_next;
    m2.px = px2_next;

    m1.py = py1_next;
    m2.py = py2_next;

    m1.x = x1_next;
    m2.x = x2_next;

    m1.y = y1_next;
    m2.y = y2_next;

    t += dt;
}

void update()
{
    update_path();
    update_pendulums();
    
    glutPostRedisplay();
}

void trace_path()
{
    glVertexPointer(2, GL_FLOAT, 0, &path1[0]);
    glColorPointer(3, GL_FLOAT, 0, &path_colors1[0]);
    glDrawArrays(GL_POINTS, 0, path1.size());
    
    glVertexPointer(2, GL_FLOAT, 0, &path2[0]);
    glColorPointer(3, GL_FLOAT, 0, &path_colors2[0]);
    glDrawArrays(GL_POINTS, 0, path2.size());
}

void draw_spring_pendulums()
{
    glBegin(GL_LINES);
    glVertex2f(0, 0);
    glVertex2f(m1.x, m1.y);
    glEnd();
    
    glBegin(GL_LINES);
    glVertex2f(m1.x, m1.y);
    glVertex2f(m2.x, m2.y);
    glEnd();

    float pendulum_radius = 0.3;
    glutSolidSphere(pendulum_radius * 0.5, 20, 20);
    
    glPushMatrix();
    {
        glTranslatef(m1.x, m1.y, 0);
        glutSolidSphere(pendulum_radius, 20, 20);
    }
    glPopMatrix();
    
    glPushMatrix();
    {
        glTranslatef(m2.x, m2.y, 0);
        glutSolidSphere(pendulum_radius, 20, 20);
    }
    glPopMatrix();
}

template<typename T>
string tostr(const T& t)
{
    ostringstream os;
    os << t;
    return os.str();
}

void draw_text()
{
    glColor3f(0, 1, 0);

    string ke_str = "KE: " + tostr(ke);
    string pe_str = "PE: " + tostr(pe);
    string total_str = "Total: " + tostr(ke + pe);
    string min_str = "Min Total: " + tostr(min_total);
    string max_str = "Max Total: " + tostr(max_total);
    string t_str = "Time: " + tostr(t);

    glRasterPos2f(-7.4,5);
    for(int i = 0; i < ke_str.length(); i++)
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, ke_str[i]);

    glRasterPos2f(-7.4,4);
    for(int i = 0; i < pe_str.length(); i++)
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, pe_str[i]);

    glRasterPos2f(3.64,5);
    for(int i = 0; i < total_str.length(); i++)
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, total_str[i]);

    glRasterPos2f(-7.4,-8.3);
    for(int i = 0; i < min_str.length(); i++)
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, min_str[i]);

    glRasterPos2f(-7.4,-9.3);
    for(int i = 0; i < max_str.length(); i++)
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, max_str[i]);

    glRasterPos2f(3.64,-9.3);
    for(int i = 0; i < t_str.length(); i++)
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, t_str[i]);
}

void key_pressed(unsigned char key, int x, int y)
{
    if(key == 'q')
    {
        exit(0);
    }
}

int main(int argc, char* argv[])
{
    if(argc != 7)
    {
        cerr << "\nERROR: Incorrect number of arguments." << endl;
        exit(1);
    }
    
    int xres = atoi(argv[1]);
    int yres = atoi(argv[2]);
    
    m1.m = 1;
    m1.x = atof(argv[3]);
    m1.y = atof(argv[4]);
    m1.px = 0;
    m1.py = 0;
    m1.k = 20;
    m1.rl = 1;
    
    m2.m = 1;
    m2.x = atof(argv[5]);
    m2.y = atof(argv[6]);
    m2.px = 0;
    m2.py = 0;
    m2.k = 20;
    m2.rl = 1;
    
    lagrangian_0 = compute_lagrangian();
    
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize(xres, yres);
    glutInitWindowPosition(0, 0);
    glutCreateWindow("Double Spring Pendulum");
    
    init();
    glutDisplayFunc(display);
    glutIdleFunc(update);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(key_pressed);
    glutMainLoop();
}
