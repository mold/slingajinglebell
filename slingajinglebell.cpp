//===========================================================================
/*
 This file is part of the CHAI 3D visualization and haptics libraries.
 Copyright (C) 2003-2010 by CHAI 3D. All rights reserved.

 This library is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License("GPL") version 2
 as published by the Free Software Foundation.

 For using the CHAI 3D libraries with software that can not be combined
 with the GNU GPL, and for taking advantage of the additional benefits
 of our support services, please contact CHAI 3D about acquiring a
 Professional Edition License.

 \author    <http://www.chai3d.org>
 \author    Francois Conti
 \author	Daniel Molin
 \author	John Brynte Turesson
 \version   (2.1.0 $Rev: 322 $) 0.1
 */
//===========================================================================

//---------------------------------------------------------------------------
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <sstream>
//---------------------------------------------------------------------------
#include "chai3d.h"

//---------------------------------------------------------------------------
// DECLARED CONSTANTS
//---------------------------------------------------------------------------

// initial size (width/height) in pixels of the display window
const int WINDOW_SIZE_W = 600;
const int WINDOW_SIZE_H = 600;

// mouse menu options (right button)
const int OPTION_FULLSCREEN = 1;
const int OPTION_SHOWSKELETON = 3;
const int OPTION_HIDESKELETON = 4;

//---------------------------------------------------------------------------
// DECLARED VARIABLES
//---------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera that renders the world in a window display
cCamera* camera;

// a light source to illuminate the objects in the virtual scene
cLight *light;

// width and height of the current window display
const int OPTION_WINDOWDISPLAY = 2;
int displayW = 0;
int displayH = 0;

// a haptic device handler
cHapticDeviceHandler* handler;

// a haptic device
cGenericHapticDevice* hapticDevice;

// force scale factor
double deviceForceScale;

// scale factor between the device workspace and cursor workspace
double workspaceScaleFactor;

// desired workspace radius of the virtual cursor
double cursorWorkspaceRadius;

// status of the main simulation haptics loop
bool simulationRunning = false;

// simulation clock
cPrecisionClock simClock;

// root resource path
string resourceRoot;

// has exited haptics simulation thread
bool simulationFinished = false;

// Limit movement in x-axis
bool limitX = false;

// show homerun
bool homerun = false;
string homerunTexts[] = { "Great!", "wow!", "HOMERUN", "You da best!!!",
		"BULL'S EYE", "KA-CHING", "*splat*" };
cLabel* titleLabel;

const double CAMERA_X = 3.8;

//---------------------------------------------------------------------------
// DECLARED MACROS
//---------------------------------------------------------------------------
// convert to resource path
#define RESOURCE_PATH(p)    (char*)((resourceRoot+string(p)).c_str())

//---------------------------------------------------------------------------
// World constants
//---------------------------------------------------------------------------
cVector3d const GRAVITY = cVector3d(0, 0, -0.00982);
cVector3d const center = cVector3d(0, 0, 0);

//---------------------------------------------------------------------------
// Slingshot
//---------------------------------------------------------------------------

// device  model
cShapeSphere* device;
double deviceRadius;

// Projectile
cVector3d projectileVel;
cShapeSphere* projectile;
double projectileRadius = 0.1;
double projectileMass = 10;
bool collided = false; // Has the projectile collided with a target???

// Slingshot
cShapeLine* slingSpringLine;
cVector3d poleTopPos(0, -0.25, 0);
cShapeLine* slingSpringLine2;
cVector3d poleTopPos2(0, 0.25, 0);
cShapeSphere* slingCenter;
cVector3d slingCenterVel(0, 0, 0);
bool springFired = false;
double prevStretch = 0;
double slingSpringConst = 30;
double slingVibrationConst = 8;
double vibrationStep = 0.001;
bool sendForce = true;

// Floor grid
const int gridLineNumber = 80;
float gridLineSpacing = 0.6;

float groundZ = -1.0;

bool keyDown = false;

double springFiredStep;

bool vibrate = true;

double deviceCenterForce = 10;
cVector3d deviceCenter;

// Target data
const int TARGETS = 3;

const double TARGET_RADIUS = 0.2;
double targetPositions[][9] = { { 10, 0, 0, 10, 0, 0, 10, 0, 0 }, // 0
		{ -3, 1, 0, -3, -1, 0, -3, 0, 0 }, // 1
		{ -3, 0.8, -0.5, -3, -0.8, 0.4, -3, 0, -0.2 }, // 2
		{ -1, 1, 0, -8, -1, 0, -4.5, 0, 0 }, // 3
		{ -7, 1, groundZ, -8, -1, groundZ, -6, 0.4, 0 }, // 4
		{ -7, 1, -0.6, -15, -2, 0.5, -10, 0.2, groundZ } // 5
};
int levels = 6;
int level = 0;

// Delay
bool delay = false;
double timer;
double SLEEP_TIME = 1;

// Data collecting
double levelTimer;
int thrownBalls;

//---------------------------------------------------------------------------
// DECLARED FUNCTIONS
//---------------------------------------------------------------------------

// callback when the window display is resized
void resizeWindow(int w, int h);

// callback when a keyboard key is pressed
void keySelect(unsigned char key, int x, int y);

// callback when the right mouse button is pressed to select a menu item
void menuSelect(int value);

// function called before exiting the application
void close(void);

// main graphics callback
void updateGraphics(void);

// main haptics loop
void updateHaptics(void);

// compute forces between tool and environment
cVector3d computeForce(const cVector3d& a_cursor, double a_cursorRadius,
		const cVector3d& a_spherePos, double a_radius, double a_stiffness);

cVector3d getVibrationForceVector(double intensity);

//////////////////////////////////////////
// Circle class
//////////////////////////////////////////
class CircleMesh {
private:
	cWorld* world;
	cVector3d pos;
	double radius;
	cMesh* circle;
	void resetGeometry();
	double rotationAngle;
	cVector3d rotationAxis;
	cMaterial mat;
public:
	CircleMesh(cWorld*, cVector3d, double);
	void setColor(double, double, double);
	void setPos(cVector3d);
	void rotate(cVector3d, double);
	void setRadius(double);
	void remove();
	virtual ~CircleMesh();
};

CircleMesh::CircleMesh(cWorld *world, cVector3d pos, double radius) {
	this->world = world;
	this->pos = pos;
	this->radius = radius;

	resetGeometry();
}

void CircleMesh::setColor(double r, double g, double b) {
	// define some material properties and apply to mesh
	cMaterial mat;
	mat.m_ambient.set(r, g, b);
	mat.m_diffuse.set(r, g, b);
	mat.m_specular.set(r, g, b);
	circle->setMaterial(mat);
	this->mat = mat;
}

void CircleMesh::setPos(cVector3d pos) {
	this->pos = pos;
	circle->setPos(pos);
}

void CircleMesh::rotate(cVector3d axis, double angle) {
	this->rotationAxis = axis;
	this->rotationAngle = angle;
	circle->rotate(axis, angle);
}

void CircleMesh::setRadius(double radius) {
	this->radius = radius;
	resetGeometry();
}

/**
 * Removes all triangles and creates them anew
 * (great when the radius has been changed for example!)
 */
void CircleMesh::resetGeometry() {
	world->removeChild(circle);
	circle = new cMesh(world);

	int res = 40;
	double step = 2 * M_PI / res;
	int v0, v1, v2;
	for (int i = 0; i < res; i++) {
		v2 = circle->newVertex(0, radius * sin(i * step), radius
				* cos(i * step));
		if (i == 0) {
			v0 = v2;
		} else if (i == 1) {
			v1 = v2;
		} else {
			circle->newTriangle(v0, v2, v1);
			v1 = v2;
		}
	}

	circle->setPos(this->pos);
	circle->rotate(this->rotationAxis, this->rotationAngle);
	circle->setMaterial(this->mat);
	circle->computeAllNormals();

	world->addChild(circle);
}

void CircleMesh::remove() {
	world->removeChild(circle);
}

CircleMesh::~CircleMesh() {

}

//////////////////////////////////////////
// Target class
//////////////////////////////////////////
class Target {
private:
	bool collided;
	cVector3d pos;
	double radius;
	CircleMesh* target;
	cShapeLine* line;
	cWorld* world;

public:
	Target(cWorld*, cVector3d, double);
	bool sphereCollide(cShapeSphere*);
	void setColor(double, double, double);
	void remove();
	bool hasCollided();
	void rotate();
    void updatePos();
    cVector3d vel;
    void setVel(cVector3d);
	virtual ~Target();
};

Target::Target(cWorld *world, cVector3d pos, double radius) {
	collided = false;
	this->world = world;
	this->pos = pos;
	this->radius = radius;
	target = new CircleMesh(world, pos, radius);
	target->setColor(0, 1, 0);
	// create a line that runs to the floor
	cVector3d floor;
	floor.copyfrom(pos);
	floor.z = groundZ;
	line = new cShapeLine(floor, pos);
	world->addChild(line);
    vel = cVector3d(0,0,0);
}

bool Target::sphereCollide(cShapeSphere *sphere) {
	double distance = (cSub(sphere->getPos(), pos)).length();
	bool col = distance < radius + sphere->getRadius();
	if (col) {
		collided = true;
		target->setColor(1, 0, 0);
	}
	return col;
}

void Target::setColor(double r, double g, double b) {
	target->setColor(r, g, b);
}

void Target::remove() {
	target->remove();
	world->removeChild(line);
}

bool Target::hasCollided() {
	return collided;
}

void Target::rotate() {
	target->rotate(cVector3d(0, 1, 0), -M_PI / 2);
}

void Target::updatePos(){
if(vel.x!=0||vel.y!=0||vel.z!=0){
    vel.add(cMul(0.001,GRAVITY));
    pos.add(vel);
    target->setPos(pos);
    target->rotate(cVector3d((double) random() / RAND_MAX,(double) random() / RAND_MAX,(double) random() / RAND_MAX),((double) random() / RAND_MAX)/50);
}
}

void Target::setVel(cVector3d nVel){
    vel = cVector3d(nVel.x, nVel.y, nVel.z);
}

Target::~Target() {

}

// Targets
Target* currentTargets[3];

// projectile shadow
CircleMesh* projectileShadowCircle;

void setLevel(int);
void setHomerun(bool);

//===========================================================================
/*
 DEMO:    GEM_membrane.cpp

 This application illustrates the use of the GEM libraries to simulate
 deformable object. In this example we load a simple mesh object and
 build a dynamic skeleton composed of volumetric spheres and 3 dimensional
 springs which model torsion, flexion and elongation properties.
 */
//===========================================================================

int main(int argc, char* argv[]) {
	//-----------------------------------------------------------------------
	// INITIALIZATION
	//-----------------------------------------------------------------------

	// parse first arg to try and locate resources
	resourceRoot = string(argv[0]).substr(0,
			string(argv[0]).find_last_of("/\\") + 1);

	//-----------------------------------------------------------------------
	// 3D - SCENEGRAPH
	//-----------------------------------------------------------------------

	// create a new world.
	world = new cWorld();

	// set the background color of the environment
	// the color is defined by its (R,G,B) components.
	world->setBackgroundColor(0.0, 0.0, 0.0);

	// create a camera and insert it into the virtual world
	camera = new cCamera(world);
	world->addChild(camera);

	// position and orient the camera
	camera->set(cVector3d(CAMERA_X, 0.0, 0.0), // camera position (eye)
			cVector3d(0.0, 0.0, 0.0), // look-at position (target)
			cVector3d(0.0, 0.0, 1.0)); // direction of the "up" vector

	// set the near and far clipping planes of the camera
	// anything in front/behind these clipping planes will not be rendered
	camera->setClippingPlanes(0.01, 100.0);

	// enable higher rendering quality because we are displaying transparent objects
	camera->enableMultipassTransparency(true);

	// create a light source and attach it to the camera
	light = new cLight(world);
	camera->addChild(light); // attach light to camera
	light->setEnabled(true); // enable light source
	light->setPos(cVector3d(2.0, 0.5, 1.0)); // position the light source
	light->setDir(cVector3d(-2.0, 0.5, 1.0)); // define the direction of the light beam

	//-----------------------------------------------------------------------
	// HAPTIC DEVICES / TOOLS
	//-----------------------------------------------------------------------

	// create a haptic device handler
	handler = new cHapticDeviceHandler();

	// get access to the first available haptic device
	handler->getDevice(hapticDevice, 0);

	// retrieve information about the current haptic device
	cHapticDeviceInfo info;
	if (hapticDevice) {
		hapticDevice->open();
		info = hapticDevice->getSpecifications();
	}

	// desired workspace radius of the cursor
	cursorWorkspaceRadius = 1.5;

	// read the scale factor between the physical workspace of the haptic
	// device and the virtual workspace defined for the tool
	workspaceScaleFactor = cursorWorkspaceRadius / info.m_workspaceRadius;

	// define a scale factor between the force perceived at the cursor and the
	// forces actually sent to the haptic device
	deviceForceScale = 0.1 * info.m_maxForce;

	// set the center point of the haptic device in the virtual environment
	deviceCenter = cVector3d(-cursorWorkspaceRadius * 0.9, 0, 0);

	// create a large sphere that represents the haptic device
	deviceRadius = 0.05;
	device = new cShapeSphere(deviceRadius);
	world->addChild(device);
	device->m_material.m_ambient.set(0.4, 0.4, 0.4, 0.7);
	device->m_material.m_diffuse.set(0.7, 0.7, 0.7, 0.7);
	device->m_material.m_specular.set(1.0, 1.0, 1.0, 0.7);
	device->m_material.setShininess(100);

	//-----------------------------------------------------------------------
	// COMPOSE THE VIRTUAL SCENE
	//-----------------------------------------------------------------------

	// A top of a pole
	cShapeSphere* poleTop = new cShapeSphere(0.03);
	poleTop->setPos(poleTopPos);
	world->addChild(poleTop);
	// A pole under a top
	cVector3d poleEnd = poleTopPos - cVector3d(0, 0, 1);
	cShapeLine* pole = new cShapeLine(poleEnd, poleTopPos);
	world->addChild(pole);
	// A sling spring line
	slingSpringLine = new cShapeLine(poleTopPos, cVector3d());
	world->addChild(slingSpringLine);

	// A top of a different pole
	cShapeSphere* poleTop2 = new cShapeSphere(0.03);
	poleTop2->setPos(poleTopPos2);
	world->addChild(poleTop2);
	// A pole under a top
	cVector3d poleEnd2 = poleTopPos2 - cVector3d(0, 0, 1);
	cShapeLine* pole2 = new cShapeLine(poleEnd2, poleTopPos2);
	world->addChild(pole2);
	// A sling spring line
	slingSpringLine2 = new cShapeLine(poleTopPos2, cVector3d());
	world->addChild(slingSpringLine2);

	slingCenter = new cShapeSphere(0.03);
	world->addChild(slingCenter);

	//////////////////////////////////////////////////////////////////////////
	// Create and add the projectile
	//////////////////////////////////////////////////////////////////////////
	projectile = new cShapeSphere(projectileRadius);
	world->addChild(projectile);
	projectile->m_material.m_ambient.set(0.4, 0.7, 0, 0.7);
	projectile->m_material.m_diffuse.set(0.5, 0.65, 0, 0.7);
	projectile->m_material.m_specular.set(1.0, 1.0, 1.0, 0.7);
	projectile->m_material.setShininess(50);

	// Shadow!
	projectileShadowCircle = new CircleMesh(world, cVector3d(0.0, 0.0, groundZ
			+ 0.0001), 0.2);
	projectileShadowCircle->rotate(cVector3d(0, 1, 0),
			-3.141592653589793238462643383 / 2.0);
	projectileShadowCircle->setColor(20, 200, 0);

	//////////////////////////////////////////////////////////////////////////
	// Create a Ground
	//////////////////////////////////////////////////////////////////////////

	// create mesh to model ground surface
	cMesh* ground = new cMesh(world);
	//world->addChild(ground);

	// create 4 vertices (one at each corner)
	double groundSizeX = 1.0;
	double groundSizeY = 1.5;

	int vertices0 = ground->newVertex(-groundSizeX, -groundSizeY, 0.0);
	int vertices1 = ground->newVertex(groundSizeX, -groundSizeY, 0.0);
	int vertices2 = ground->newVertex(groundSizeX, groundSizeY, 0.0);
	int vertices3 = ground->newVertex(-groundSizeX, groundSizeY, 0.0);

	// compose surface with 2 triangles
	ground->newTriangle(vertices0, vertices1, vertices2);
	ground->newTriangle(vertices0, vertices2, vertices3);

	// compute surface normals
	ground->computeAllNormals();

	// position ground at the right level
	ground->setPos(0.0, 0.0, groundZ);

	//////////////////////////////////////////////////////////////////////////
	// Create a floor grid matrix (good ol' fashioned)
	//////////////////////////////////////////////////////////////////////////

	float cAlpha = 0.5;
	for (int i = 0; i < gridLineNumber; i++) {
		float y = i * gridLineSpacing - gridLineSpacing * gridLineNumber / 2.0;
		float z = groundZ + 0.00001;
		float x = gridLineSpacing * gridLineNumber / 2.0;

		cShapeLine* line1 = new cShapeLine(cVector3d(-x, y, z), cVector3d(x, y,
				z));

		line1->m_ColorPointA.set(133, 0, 137, cAlpha);
		line1->m_ColorPointB.set(0, 165, 165, cAlpha);
		world->addChild(line1);

		for (int j = 0; j < gridLineNumber; j++) {
			float y = j * gridLineSpacing - gridLineSpacing * gridLineNumber
					/ 2.0;

			cShapeLine* line2 = new cShapeLine(cVector3d(y, -x, z), cVector3d(
					y, x, z));
			line2->m_ColorPointA.set(133, 0, 137, cAlpha);
			line2->m_ColorPointB.set(0, 165, 165, cAlpha);

			world->addChild(line2);
		}
	}

	//-----------------------------------------------------------------------
	// OPEN GL - WINDOW DISPLAY
	//-----------------------------------------------------------------------

	// initialize GLUT
	glutInit(&argc, argv);

	// retrieve the resolution of the computer display and estimate the position
	// of the GLUT window so that it is located at the center of the screen
	int screenW = glutGet(GLUT_SCREEN_WIDTH);
	int screenH = glutGet(GLUT_SCREEN_HEIGHT);
	int windowPosX = (screenW - WINDOW_SIZE_W) / 2;
	int windowPosY = (screenH - WINDOW_SIZE_H) / 2;

	// initialize the OpenGL GLUT window
	glutInitWindowPosition(windowPosX, windowPosY);
	glutInitWindowSize(WINDOW_SIZE_W, WINDOW_SIZE_H);
	glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
	glutCreateWindow(argv[0]);
	glutDisplayFunc(updateGraphics);
	glutKeyboardFunc(keySelect);
	glutReshapeFunc(resizeWindow);
	glutSetWindowTitle("CHAI 3D");

	// create a mouse menu (right button)
	glutCreateMenu(menuSelect);
	glutAddMenuEntry("full screen", OPTION_FULLSCREEN);
	glutAddMenuEntry("window display", OPTION_WINDOWDISPLAY);
	glutAddMenuEntry("show skeleton", OPTION_SHOWSKELETON);
	glutAddMenuEntry("hide skeleton", OPTION_HIDESKELETON);
	glutAttachMenu(GLUT_RIGHT_BUTTON);

	//-----------------------------------------------------------------------
	// START SIMULATION
	//-----------------------------------------------------------------------

	// initialize the first level
	setLevel(-1);

	// simulation in now running
	simulationRunning = true;

	// create a thread which starts the main haptics rendering loop
	cThread* hapticsThread = new cThread();
	hapticsThread->set(updateHaptics, CHAI_THREAD_PRIORITY_HAPTICS);

	// start the main graphics rendering loop
	glutMainLoop();

	// close everything
	close();

	// exit
	return (0);
}

//---------------------------------------------------------------------------

void setNextLevel() {
	if (level == levels) {
		// Do something when the game has ended
	} else {
		setLevel(level + 1);
	}
}

//---------------------------------------------------------------------------

void setLevel(int lvl) {
	setHomerun(false);

	// Remove any existing targets
	if (lvl != -1) {
		for (int i = 0; i < TARGETS; i++) {
			currentTargets[i]->remove();
		}
	}

	level = lvl;
	if (level >= levels || level < 0) {
		level = 0;
	}

	// Initialize everything
	for (int i = 0; i < TARGETS; i++) {
		currentTargets[i] = new Target(world, cVector3d(
				targetPositions[level][i * 3],
				targetPositions[level][i * 3 + 1], targetPositions[level][i * 3
						+ 2]), TARGET_RADIUS);
		if (targetPositions[level][i * 3 + 2] == groundZ) {
			currentTargets[i]->rotate();
		}
	}
	projectileVel.zero();
	projectile->setPos(cVector3d(0, 0, groundZ));

	// Reset timer and counter
	levelTimer = 0;
	thrownBalls = 0;
}

//---------------------------------------------------------------------------

void resizeWindow(int w, int h) {
	// update the size of the viewport
	displayW = w;
	displayH = h;
	glViewport(0, 0, displayW, displayH);
}

//---------------------------------------------------------------------------

void keySelect(unsigned char key, int x, int y) {
	// escape key
	if ((key == 27) || (key == 'x')) {
		// close everything
		close();

		// exit application
		exit(0);
	} else if (key == '1') {
		limitX = !limitX;
		std::cout << "limitx: " << limitX << std::endl;
	} else if (key == 'v') {
		vibrate = !vibrate;
		std::cout << "vibrate: " << vibrate << std::endl;
	} else if (key == 'h') {
		// HOMERUUUN
		setHomerun(!homerun);
	} else if (key == 'n') {
		setNextLevel();
	} else if (key == 'f') {
		sendForce = !sendForce;
		std::cout << "sendforce: " << sendForce << std::endl;
	}
}

void setHomerun(bool home) {
	homerun = home;
	if (homerun) {
		// create a small label as title
		titleLabel = new cLabel();

		cFont* font = cFont::createFont();
		font->setPointSize(200);
		font->setFontFace("Monospace");
		font = cFont::createFont(font);

		// define its position, color and string message, and font
		titleLabel->setPos(0, 0, 0);
		titleLabel->m_fontColor.set((double) random() / RAND_MAX,
				(double) random() / RAND_MAX, (double) random() / RAND_MAX);
		titleLabel->m_string = homerunTexts[level];
		titleLabel->m_font = font;

		world->addChild(titleLabel);
	} else {
		world->removeChild(titleLabel);
	}
}

//---------------------------------------------------------------------------

void menuSelect(int value) {
	switch (value) {
	// enable full screen display
	case OPTION_FULLSCREEN:
		glutFullScreen();
		break;

		// reshape window to original size
	case OPTION_WINDOWDISPLAY:
		glutReshapeWindow(WINDOW_SIZE_W, WINDOW_SIZE_H);
		break;

	}
}

//---------------------------------------------------------------------------

void close(void) {
	// stop the simulation
	simulationRunning = false;

	// wait for graphics and haptics loops to terminate
	while (!simulationFinished) {
		cSleepMs(100);
	}

	// close haptic device
	hapticDevice->close();
}

//---------------------------------------------------------------------------

void updateGraphics(void) {
	if (homerun) {
		titleLabel->setPos(projectile->getPos());
		titleLabel->m_fontColor.set((double) random() / RAND_MAX,
				(double) random() / RAND_MAX, (double) random() / RAND_MAX);

	}



	// update shadow size and position
	projectileShadowCircle->setPos(cVector3d(projectile->getPos().x,
			projectile->getPos().y, groundZ + 0.0001));
	projectileShadowCircle->setRadius(projectileRadius
			/ (projectile->getPos().z + 2));

	// render world
	camera->renderView(displayW, displayH);

	// Swap buffers
	glutSwapBuffers();

	// check for any OpenGL errors
	GLenum err;
	err = glGetError();
	if (err != GL_NO_ERROR)
		printf("Error:  %s\n", gluErrorString(err));

	// inform the GLUT window to call updateGraphics again (next frame)
	if (simulationRunning) {
		glutPostRedisplay();
	}
}

//---------------------------------------------------------------------------

void updateHaptics(void) {
	// reset clock
	simClock.reset();

	// main haptic simulation loop
	while (simulationRunning) {
		// stop the simulation clock
		simClock.stop();

		// read the time increment in seconds
		double timeInterval = simClock.getCurrentTimeSeconds();
		if (timeInterval > 0.001) {
			timeInterval = 0.001;
		}

		// restart the simulation clock
		simClock.reset();
		simClock.start();

		// Update level timer
		levelTimer += timeInterval;

		// init temp variable
		cVector3d force;
		force.zero();

		cVector3d realPos;
		cVector3d pos;
		cVector3d virtualPos;
		hapticDevice->getPosition(realPos);
		realPos.mul(workspaceScaleFactor);
		pos.copyfrom(realPos);
		if (limitX) {
			pos.x = 0;
		}
		if (pos.z < groundZ) {
			pos.z = groundZ;
		}

		// Update the camera
		// position and orient the camera
		camera->set(cVector3d(CAMERA_X, pos.y / 6, pos.z / 6), // camera position (eye)
				cVector3d(0.0, 0.0, 0.0), // look-at position (target)
				cVector3d(0.0, 0.0, 1.0)); // direction of the "up" vector

		virtualPos = cAdd(center, cSub(pos, deviceCenter));
		device->setPos(virtualPos);

		// Get vector from projectile to slingtop
		cVector3d spring = cNegate(virtualPos);
		double stretch = spring.length();
		double stretchStep = stretch - prevStretch;
		if (stretchStep < 0) {
			stretchStep = -stretchStep;
		}
		spring.normalize();

		double vibrationIntensity = 0.0;

		bool key;
		hapticDevice->getUserSwitch(0, key);
		if (key && !delay) {
			keyDown = true;
			collided = false;
			springFired = false;
			// Set the projectile virutal position
			projectile->setPos(virtualPos);
			projectileVel = cVector3d(0, 0, 0);

			slingCenter->setPos(virtualPos);

			/* Activate spring */

			// Add spring force to allaround force
			force.add(cAdd(cMul(slingSpringConst * stretch, spring), force));

			// Add vibration
			if (vibrate) {
				vibrationIntensity = (1 - cos(M_PI * stretch / 2)) / 2;
				//vibrationIntensity = pow(stretch / 2, 3);
				if (stretchStep < vibrationStep) {
					vibrationIntensity /= 5;
				}
				force.add(getVibrationForceVector(vibrationIntensity));
			}

			// add gravity to haptic device
			// TODO: This does not give the required effect
			//cVector3d projectileGravity = cMul(projectileMass * 30, GRAVITY);
			//force.add(projectileGravity);
		} else if (keyDown) {
			// The key has been released
			keyDown = false;

			springFired = true;
			projectileVel = cVector3d(0, 0, 0);
			springFiredStep = 100000000; // ååh förlååååt förlååååååååt!!!

			thrownBalls++;

		} else {
			// Add gravitational force/acceleration to projectile - it's flying away bro
			cVector3d gravityStep = cMul(timeInterval, GRAVITY);
			projectileVel.add(gravityStep);

			// Pull the device to its initial position
			cVector3d slingCenterPos = slingCenter->getPos();
			cVector3d slingCenterAcc = cNegate(slingCenterPos);
			slingCenterVel.add(cMul(timeInterval, slingCenterAcc));
			double stiffness = slingCenterVel.length() * 0.8;
			slingCenterVel.sub(cMul(stiffness, slingCenterVel));
			slingCenterPos.add(slingCenterVel);
			slingCenter->setPos(slingCenterPos);

			// Pull the device towards the center
			force.add(cAdd(cMul(deviceCenterForce * stretch, spring), force));

			// make projectile stick to ground
			cVector3d projPos = projectile->getPos();
			if (projPos.z + projectileVel.z < groundZ) {
				cVector3d dir = cNormalize(projectileVel);
				double zDistToGround = (groundZ - projPos.z) / dir.z;
				projectile->setPos(cAdd(projPos, cMul(zDistToGround, projectileVel)));
                projectileVel.z = -projectileVel.z*0.8;
projectileVel.x = projectileVel.x*0.9;
projectileVel.y = projectileVel.y*0.9;
			}
		}

		if (springFired) {
			cVector3d projectilePos = projectile->getPos();

			double length = cSub(projectilePos, center).length();
			if (length < springFiredStep) {
				springFiredStep = length;

				// Get vector from projectile to slingtop
				cVector3d acc = poleTopPos - projectile->getPos();
				double distance = acc.length() * slingSpringConst / 2;
				cVector3d springForce = cDiv(projectileMass, acc);
				// apply the force to the sling force
				springForce.mul(timeInterval);
				projectileVel.add(springForce);

				// Get another vector from projectile to another slingtop
				acc = poleTopPos2 - projectile->getPos();
				distance = acc.length() * slingSpringConst / 2;
				springForce = cDiv(projectileMass, acc);
				// apply the second force to the sling force
				springForce.mul(timeInterval);
				projectileVel.add(springForce);

				slingCenterVel.copyfrom(projectileVel);
			} else {
				springFired = false;
			}

		}

		// Update the slingshot graphcis
		slingSpringLine->m_pointB = slingCenter->getPos();
		slingSpringLine2->m_pointB = slingCenter->getPos();

		// update position of projectile (shadow moves in updateGraphics)
		projectile->setPos(cAdd(projectile->getPos(), projectileVel));

		/** PRINT INFO **/
		/*string posStr;
		 string velStr;
		 projectile->getPos().str(posStr);
		 projectileVel.str(velStr);
		 std::cout << "pos: " << posStr << " | vel: " << projectileVel.z << std::endl;*/

		// scale force
		force.mul(deviceForceScale);
		if (limitX) {
			// restrict movement in x-axis
			//force.x = -realPos.x * 200;
		}

		// send forces to haptic device
		if (sendForce) {
			hapticDevice->setForce(force);
		} else {
			cVector3d zero(0, 0, 0);
			hapticDevice->setForce(zero);
		}

		// Check collision with targets
		bool collision = true;
		for (int i = 0; i < TARGETS; i++) {
			currentTargets[i]->sphereCollide(projectile);
			collision = collision && currentTargets[i]->hasCollided();
		}
		if (collision && !delay) {
			// SUCCESS
			setHomerun(true);
			delay = true;
			timer = 0;

			printf("%i\t%f\t%i\n", level, levelTimer, thrownBalls);
		}
		if (!collided) {
			for (int i = 0; i < 3; i++) {
				if (currentTargets[i]->sphereCollide(projectile)) {
                    currentTargets[i]->setVel(projectileVel);
					projectileVel = cVector3d(-projectileVel.x*0.6	,projectileVel.y*0.6,projectileVel.z*0.6);
					collided = true;

				}
			}
		}

    // move targets
    for(int i = 0; i < TARGETS; i++){
        currentTargets[i]->updatePos();
    }


		// check the delay
		if (delay) {
			timer += timeInterval;
			if (timer > SLEEP_TIME) {
				delay = false;
				setNextLevel();
			}
		}

		prevStretch = stretch;
	}

	// exit haptics thread
	simulationFinished = true;
}

//---------------------------------------------------------------------------

cVector3d computeForce(const cVector3d& a_cursor, double a_cursorRadius,
		const cVector3d& a_spherePos, double a_radius, double a_stiffness) {

	// compute the reaction forces between the tool and the ith sphere.
	cVector3d force;
	force.zero();
	cVector3d vSphereCursor = a_cursor - a_spherePos;

	// check if both objects are intersecting
	if (vSphereCursor.length() < 0.0000001) {
		return (force);
	}

	if (vSphereCursor.length() > (a_cursorRadius + a_radius)) {
		return (force);
	}

	// compute penetration distance between tool and surface of sphere
	double penetrationDistance = (a_cursorRadius + a_radius)
			- vSphereCursor.length();
	cVector3d forceDirection = cNormalize(vSphereCursor);
	force = cMul(penetrationDistance * a_stiffness, forceDirection);

	// return result
	return (force);
}

cVector3d getVibrationForceVector(double intensity) {
	if (intensity > 1)
		intensity = 1;
	else if (intensity < 0)
		intensity = 0;

	return cVector3d(((double) random() / RAND_MAX) * intensity
			* slingVibrationConst, ((double) random() / RAND_MAX) * intensity
			* slingVibrationConst, ((double) random() / RAND_MAX) * intensity
			* slingVibrationConst);
}
