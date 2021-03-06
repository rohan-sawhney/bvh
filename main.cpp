#ifdef __APPLE_CC__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include "ZFighter.h"
#include "Mesh.h"

int gridX = 600;
int gridY = 600;
int gridZ = 600;

const double fovy = 50.;
const double clipNear = .01;
const double clipFar = 1000.0;
double x = 0;
double z = -2.5;
double y = 0;

Mesh mesh;
ZFighter zFighter;
std::string path = "/Users/rohansawhney/Desktop/developer/C++/bvh/geometry.obj";
bool success = true;

void printInstructions()
{
    std::cerr << "' ': toggle between z fighting"
              << "↑/↓: move in/out\n"
              << "w/s: move up/down\n"
              << "a/d: move left/right\n"
              << "r: reload\n"
              << "escape: exit program\n"
              << std::endl;
}

void init()
{
    glClearColor(0.0, 0.0, 0.0, 0.0);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_DEPTH_TEST);
}

void drawFaces()
{
    for (FaceIter f = mesh.faces.begin(); f != mesh.faces.end(); f++) {

        glColor4f(0, 0, 1, 0.6);
        glBegin(GL_TRIANGLES);
        
        glVertex3d(mesh.vertices[f->indices[0]].position.x(),
                   mesh.vertices[f->indices[0]].position.y(),
                   mesh.vertices[f->indices[0]].position.z());
        glVertex3d(mesh.vertices[f->indices[1]].position.x(),
                   mesh.vertices[f->indices[1]].position.y(),
                   mesh.vertices[f->indices[1]].position.z());
        glVertex3d(mesh.vertices[f->indices[2]].position.x(),
                   mesh.vertices[f->indices[2]].position.y(),
                   mesh.vertices[f->indices[2]].position.z());
        
        glEnd();
        
        glColor4f(1, 1, 1, 0.6);
        glBegin(GL_LINE_LOOP);
        
        glVertex3d(mesh.vertices[f->indices[0]].position.x(),
                   mesh.vertices[f->indices[0]].position.y(),
                   mesh.vertices[f->indices[0]].position.z());
        glVertex3d(mesh.vertices[f->indices[1]].position.x(),
                   mesh.vertices[f->indices[1]].position.y(),
                   mesh.vertices[f->indices[1]].position.z());
        glVertex3d(mesh.vertices[f->indices[2]].position.x(),
                   mesh.vertices[f->indices[2]].position.y(),
                   mesh.vertices[f->indices[2]].position.z());
        
        glEnd();
    }
}

void display()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    
    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);
    double aspect = (double)viewport[2] / (double)viewport[3];
    gluPerspective(fovy, aspect, clipNear, clipFar);
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    gluLookAt(0, 0, z, x, y, 0, 0, 1, 0);
    
    if (success) {
        drawFaces();
    }
    
    glutSwapBuffers();
}

void keyboard(unsigned char key, int x0, int y0)
{
    switch (key) {
        case 27:
            exit(0);
        case ' ':
            zFighter.process(mesh);
            break;
        case 'a':
            x -= 0.03;
            break;
        case 'd':
            x += 0.03;
            break;
        case 'w':
            y += 0.03;
            break;
        case 's':
            y -= 0.03;
            break;
        case 'r':
            success = mesh.read(path);
            break;
    }
    
    glutPostRedisplay();
}

void special(int i, int x0, int y0)
{
    switch (i) {
        case GLUT_KEY_UP:
            z += 0.03;
            break;
        case GLUT_KEY_DOWN:
            z -= 0.03;
            break;
    }
    
    glutPostRedisplay();
}

int main(int argc, char** argv) {

    success = mesh.read(path);
    
    printInstructions();
    glutInitWindowSize(gridX, gridY);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
    glutInit(&argc, argv);
    glutCreateWindow("Bounding Volume Hierarchy");
    init();
    glutDisplayFunc(display);
    glutKeyboardFunc(keyboard);
    glutSpecialFunc(special);
    glutMainLoop();
    
    return 0;
}
