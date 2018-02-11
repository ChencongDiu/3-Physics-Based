#include "stdafx.h"

// standard
#include <assert.h>
#include <math.h>
#include <iostream>

// glut
#define GLUT_DISABLE_ATEXIT_HACK
#include <gl/glut.h>

using namespace std;
#define pi 3.1415926

//================================
// global variables
//================================
// screen size
int g_screenWidth = 0;
int g_screenHeight = 0;

// frame index
int g_frameIndex = 0;

// angle for rotation
int g_angle = 0;
double g_xdistance = 0.0;
double g_ydistance = 0.0;

// balls parameters
GLfloat radius = 0.2f; //balls radius
int numberBalls = 15; //number of balls
GLfloat curPositions[15][3] = { 0 }; //positions of balls of current frame
GLfloat nextPositions[15][3] = { 0 }; //positions of balls of next frame
GLfloat curVelocity[15][3] = { 0 }; //velocities of balls of current frame
GLfloat nextVelocity[15][3] = { 0 }; //velocities of balls of next frame
GLfloat ballsM[15][16] = { 0 }; //balls matrix

// environment parameters
GLfloat t = 0.025f; //time interval
GLfloat g = -2.5f; //gravity
GLfloat f = 0.75f; //collision coefficient

//================================
// Print String on Screen
//================================
void drawBitmapText(char *string, float x, float y, float z)
{
	char *c;
	glRasterPos3f(x, y, z);
	for (c = string; *c != '\0'; c++)
	{
		glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, *c);
	}
}

//================================
// initialize balls matrix ballsM with initPositions
//================================
void init(void) {
	//initialize initPositions with random positions
	//and velocities of balls
	for (int i = 0; i < numberBalls; i++) {
		curPositions[i][0] = 3.0f + (static_cast <float> (rand()) / static_cast <float> (RAND_MAX) * 4.0f); //x
		curPositions[i][1] = 1.0f + (static_cast <float> (rand()) / static_cast <float> (RAND_MAX) * 2.0f); //y
		curPositions[i][2] = 3.0f + (static_cast <float> (rand()) / static_cast <float> (RAND_MAX) * 4.0f); //z
		curVelocity[i][0] = -2.0f + (static_cast <float> (rand()) / static_cast <float> (RAND_MAX) * 2.0f); //v_x
		curVelocity[i][1] = -2.0f + (static_cast <float> (rand()) / static_cast <float> (RAND_MAX) * 2.0f); //v_y
		curVelocity[i][2] = -2.0f + (static_cast <float> (rand()) / static_cast <float> (RAND_MAX) * 2.0f); //v_z
		ballsM[i][15] = 1.0f;
		for (int j = 0; j < 3; j++) {
			ballsM[i][5 * j] = 1.0f;
			ballsM[i][12 + j] = curPositions[i][j];
		}
	}
}

// generate floor
void floorGenerator() {
	glBegin(GL_LINES);
	for (int i = -100;i <= 100;i += 1) {
		glVertex3f(i, 0, -100);
		glVertex3f(i, 0, 100);
		glVertex3f(-100, 0, i);
		glVertex3f(100, 0, i);
	};
	glEnd();
}

// floor collision detection
// once hit the floor v1 = -v0 * f
void floorCollision(int index) {
	//hit the floor
	if (curPositions[index][1] < radius) {
		curVelocity[index][1] = -f * curVelocity[index][1]; //reverse the velocity on y-axis
	}
}

//euler distance between two points
GLfloat distance(GLfloat pos1[3], GLfloat pos2[3]) {
	return sqrt((pos1[0] - pos2[0]) * (pos1[0] - pos2[0]) +
				(pos1[1] - pos2[1]) * (pos1[1] - pos2[1]) +
				(pos1[2] - pos2[2]) * (pos1[2] - pos2[2]));
}

// normalization
void norm(GLfloat m[3]) {
	GLfloat sumsqrt = m[0] * m[0] + m[1] * m[1] + m[2] * m[2];
	if (sumsqrt != 0) // avoid being divided by 0
	{
		GLfloat base = sqrt(sumsqrt);
		m[0] = m[0] / base;
		m[1] = m[1] / base;
		m[2] = m[2] / base;
	}
}

// vector dot multiple
GLfloat vectorDotMult(GLfloat v1[3], GLfloat v2[3]) {
	return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

// balls collision detection
// once hit, velocities along the center line of two balls reversed
void ballsCollision(int index) {
	for (int i = index + 1; i < numberBalls; i++) {
		//hit other balls
		if (distance(curPositions[index], curPositions[i]) < 2 * radius) {
			GLfloat origin1[3] = { 0 }; //current ball as origin
			GLfloat origin2[3] = { 0 }; //target ball as origin
			for (int j = 0; j < 3; j++) {
				origin1[j] = curPositions[i][j] - curPositions[index][j];
				origin2[j] = curPositions[index][j] - curPositions[i][j];
			}
			norm(origin1);
			norm(origin2);
			GLfloat temp1 = vectorDotMult(origin1, curVelocity[index]);
			GLfloat temp2 = vectorDotMult(origin2, curVelocity[i]);
			GLfloat ux1, uy1, ux2, uy2, v1, v2;
			for (int j = 0; j < 3; j++) {
				ux1 = temp1 * origin1[j];
				uy1 = curVelocity[index][j] - ux1;
				ux2 = temp2 * origin2[j];
				uy2 = curVelocity[i][j] - ux2;
				v1 = (ux1 + ux2 - (ux1 - ux2)) / 2.0f;
				v2 = (ux1 + ux2 - (ux1 - ux1)) / 2.0f;
				curVelocity[index][j] = v1 + uy1;
				curVelocity[i][j] = v2 + uy2;
			}
		}
	}
}

// interpolater, for next frame of a certain ball
// collision detection
// velocity change
void interpolater(int index) {
	for (int i = 0; i < 3; i++) {
		//v1 = v0 + at, s1 = s0 + v1t
		nextVelocity[index][i] = curVelocity[index][i] + (i == 1 ? g * t : 0);
		curVelocity[index][i] = nextVelocity[index][i];
		nextPositions[index][i] = curPositions[index][i] + nextVelocity[index][i] * t;
		curPositions[index][i] = nextPositions[index][i];
		ballsM[index][12 + i] = nextPositions[index][i];
	}
	floorCollision(index);
	ballsCollision(index);
}

// render balls
void ballsGenerator() {
	for (int i = 0; i < numberBalls; i++) {
		glPushMatrix();
		interpolater(i);
		glMultMatrixf(ballsM[i]);
		glutSolidSphere(radius, 25, 25);
		glPopMatrix();
	}
}

//================================
// render
//================================
void render(void) {
	// clear buffer
	glClearColor(0.5, 0.5, 0.5, 0.7);
	glClearDepth(1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// render state
	glEnable(GL_DEPTH_TEST);
	glShadeModel(GL_SMOOTH);

	// modelview matrix
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(0.0f, 0.0f, -5.0f);
	gluLookAt(0.0f, 3.0f, 0.0f, 3.0f, 0.0f, 3.0f, 0.0f, 1.0f, 0.0f);

	//helper and 6 points
	drawBitmapText("Press Spacebar to Exit", 13.0, 2.5, 0.0);

	// enable lighting
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	// light source attributes
	GLfloat LightAmbient[] = { 0.4f, 0.4f, 0.4f, 1.0f };
	GLfloat LightDiffuse[] = { 0.3f, 0.3f, 0.3f, 1.0f };
	GLfloat LightSpecular[] = { 0.4f, 0.4f, 0.4f, 1.0f };
	GLfloat LightPosition[] = { 5.0f, 5.0f, 5.0f, 1.0f };

	glLightfv(GL_LIGHT0, GL_AMBIENT, LightAmbient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, LightDiffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, LightSpecular);
	glLightfv(GL_LIGHT0, GL_POSITION, LightPosition);

	// surface material attributes
	GLfloat material_Ka[] = { 0.11f, 0.06f, 0.11f, 1.0f };
	GLfloat material_Kd[] = { 0.43f, 0.47f, 0.54f, 1.0f };
	GLfloat material_Ks[] = { 0.33f, 0.33f, 0.52f, 1.0f };
	GLfloat material_Ke[] = { 0.1f , 0.0f , 0.1f , 1.0f };
	GLfloat material_Se = 10;

	glMaterialfv(GL_FRONT, GL_AMBIENT, material_Ka);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, material_Kd);
	glMaterialfv(GL_FRONT, GL_SPECULAR, material_Ks);
	glMaterialfv(GL_FRONT, GL_EMISSION, material_Ke);
	glMaterialf(GL_FRONT, GL_SHININESS, material_Se);

	// render ground and balls
	floorGenerator();
	ballsGenerator();

	// disable lighting
	glDisable(GL_LIGHT0);
	glDisable(GL_LIGHTING);

	// swap back and front buffers
	glutSwapBuffers();
}

//================================
// keyboard input
//================================
void keyboard(unsigned char key, int x, int y) {
	switch (key) {
	case ' ':
		exit(0);
		break;
	default:
		break;
	}
	glutPostRedisplay();
}

//================================
// reshape : update viewport and projection matrix when the window is resized
//================================
void reshape(int w, int h) {
	// screen size
	g_screenWidth = w;
	g_screenHeight = h;

	// viewport
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);

	// projection matrix
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0, (GLfloat)w / (GLfloat)h, 1.0, 2000.0);
}

//================================
// timer : triggered every 16ms ( about 60 frames per second )
//================================
void timer(int value) {
	// render
	glutPostRedisplay();

	// increase frame index
	g_frameIndex++;

	// reset timer
	// 16 ms per frame ( about 60 frames per second )
	glutTimerFunc(16, timer, 0);
}

//================================
// main
//================================
int main(int argc, char** argv) {
	// create opengL window
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(1600, 900);
	glutInitWindowPosition(50, 50);
	glutCreateWindow("Lab3");

	// init
	init();

	// set callback functions
	glutDisplayFunc(render);
	glutReshapeFunc(reshape);
	glutKeyboardFunc(keyboard);
	glutTimerFunc(16, timer, 0);

	// main loop
	glutMainLoop();

	return 0;
}