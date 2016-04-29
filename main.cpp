#define OUTPUT_ANIMATION 1

#include <stdlib.h>
#include <stdio.h>

#include "Particles.h"

#if OUTPUT_ANIMATION
#include <opencv2/opencv.hpp>
#endif


inline float clip(const float& n, const float& lower, const float& upper) 
{
    return glm::max(lower, glm::min(n, upper));
}

float cube_width = 2;
float cube_height = 2;
float cube_length = 2;
float most_bottom[] = {-1.0 , -1.0, -1.0};

float ver[8][3] = 
{
    {most_bottom[0],most_bottom[1],most_bottom[2] + cube_length},
    {most_bottom[0],most_bottom[1] + cube_height,most_bottom[2] + cube_length},
    {most_bottom[0] + cube_width,most_bottom[1] + cube_height,most_bottom[2] + cube_length},
    {most_bottom[0] + cube_width,most_bottom[1],most_bottom[2] + cube_length},
    {most_bottom[0],most_bottom[1],most_bottom[2]},
    {most_bottom[0],most_bottom[1] + cube_height,most_bottom[2]},
    {most_bottom[0] + cube_width,most_bottom[1] + cube_height,most_bottom[2]},
    {most_bottom[0] + cube_width,most_bottom[1],most_bottom[2]},
};

float theta = M_PI/8;
float phi = -M_PI/8+M_PI_2;
float dist = 2.5;
int width = 800;
int height = 800;
int frame = 0;
const int render_step = 3;
int mx, my;

Particles particles = Particles(most_bottom, cube_width, cube_length, cube_height);

void display(void);

void reshape(int width, int height);

void quad(int a,int b,int c,int d)
{
    glBegin(GL_QUADS);
    glVertex3fv(ver[a]);
    glVertex3fv(ver[b]);
    glVertex3fv(ver[c]);
    glVertex3fv(ver[d]);
    glEnd();
}
void cube(){
    quad(0,3,2,1);
    quad(2,3,7,6);
    quad(0,4,7,3);
    quad(1,2,6,5);
    quad(4,5,6,7);
    quad(0,1,5,4);

}
void idle(void)
{
    particles.step();
    glutPostRedisplay();
    if(frame/render_step >= 300)
        return;
    if(frame%render_step == 0)
    {
        #if OUTPUT_ANIMATION
        cv::Mat3b image(height, width);
        glReadPixels(0, 0, width, height, GL_BGR, GL_UNSIGNED_BYTE, image.data);
        cv::flip(image, image, 0);
        char fn[512];
        sprintf(fn, "result/%04d.png", frame/render_step);
        cv::imwrite(fn, image);
        #endif
    }
    frame++;
}

void mouse(int button, int state, int x, int y);

void motion(int x, int y);

void keyboard(unsigned char c, int x, int y)
{
    switch(c)
    {
    case 'o' :
        break;
    }
}

int main(int argc, char** argv)
{
    glutInit(&argc, argv);

    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
    glutInitWindowSize(width, height);

    (void)glutCreateWindow("GLUT Program");
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutIdleFunc(idle);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutMainLoop();
    glutKeyboardFunc(keyboard);

    return EXIT_SUCCESS;
}

void reshape(int w, int h)
{
    width = w;
    height = h;
    glViewport(0, 0, w, h);
}

void display(void)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // your drawing code goes here
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(90, 1, 0.01, 100);
    gluLookAt(dist*sin(phi)*cos(theta), dist*cos(phi), dist*sin(phi)*sin(theta),
            0, 0, 0, 
            0, 1, 0);
    glEnable(GL_DEPTH_TEST);
    glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
    cube();
    glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
    particles.render();
    glutSwapBuffers();
}

void mouse(int button, int state, int x, int y)
{
    if(button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
    {
        mx = x;
        my = y;
    }
}

void motion(int x, int y)
{
    int dx = x - mx;
    int dy = y - my;
    mx = x;
    my = y;
    if(abs(dx) > abs(dy))
        theta += dx*0.005;
    else
        phi -= dy*0.005;
    if(theta > 2*M_PI)
        theta -= 2*M_PI;
    if(theta < -2*M_PI)
        theta += 2*M_PI;
    phi = clip(phi, M_PI/12, M_PI*11/12);
    glutPostRedisplay();
}
