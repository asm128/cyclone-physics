// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include "cyclone.h"
#include "ogl_headers.h"
#include "app.h"
#include "timing.h"

#include <stdio.h>
#include <cassert>

// The main demo class definition.
class SailboatDemo : public Application
{
    cyclone::Buoyancy		buoyancy;

    cyclone::Aero			sail;
    cyclone::RigidBody		sailboat;
    cyclone::ForceRegistry	registry;

    cyclone::Random			r;
    cyclone::Vector3		windspeed;

    float					sail_control;

public:
	virtual					~SailboatDemo	()						{}
							SailboatDemo	();

	virtual const char*		GetTitle		();						// Returns the window title for the demo. 
    virtual void			Display			();						// Display the particles. 
    virtual void			Update			();						// Update the particle positions. 
    virtual void			Key				(unsigned char key);	// Handle a key press. 

	virtual void			Mouse			(int, int, int, int)	{}	// Called when GLUT detects a mouse button press.
	virtual void			MouseDrag		(int, int)				{}	// Called when GLUT detects a mouse drag.
};

// Method definitions
SailboatDemo::SailboatDemo()
	: Application()
	, sail			(cyclone::Matrix3(0,0,0, 0,0,0, 0,0,-1.0f), {2.0f, 0, 0}, &windspeed)
	, buoyancy		({0.0f, 0.5f, 0.0f}, 1.0f, 3.0f, 1.6f)
	, sail_control	(0)
	, windspeed		({})
{
    // Set up the boat's rigid body.
	sailboat.Position = {0, 1.6f, 0};
    sailboat.setOrientation(1,0,0,0);

    sailboat.Velocity = {};
    sailboat.Rotation = {};

    sailboat.setMass(200.0f);
    cyclone::Matrix3 it;
	it.setBlockInertiaTensor({2, 1, 1}, 100.0f);
    sailboat.setInertiaTensor(it);

    sailboat.setDamping(0.8f, 0.8f);

    sailboat.Acceleration = cyclone::Vector3::GRAVITY;
    sailboat.calculateDerivedData();

    sailboat.setAwake();
    sailboat.setCanSleep(false);

    registry.Registrations.push_back({&sailboat, &sail		});
    registry.Registrations.push_back({&sailboat, &buoyancy	});
}

static void drawBoat()
{
    // Left Hull
    glPushMatrix();
    glTranslatef(0, 0, -1.0f);
    glScalef(2.0f, 0.4f, 0.4f);
    glutSolidCube(1.0f);
    glPopMatrix();

    // Right Hull
    glPushMatrix();
    glTranslatef(0, 0, 1.0f);
    glScalef(2.0f, 0.4f, 0.4f);
    glutSolidCube(1.0f);
    glPopMatrix();

    // Deck
    glPushMatrix();
    glTranslatef(0, 0.3f, 0);
    glScalef(1.0f, 0.1f, 2.0f);
    glutSolidCube(1.0f);
    glPopMatrix();

    // Mast
    glPushMatrix();
    glTranslatef(0, 1.8f, 0);
    glScalef(0.1f, 3.0f, 0.1f);
    glutSolidCube(1.0f);
    glPopMatrix();

}

void SailboatDemo::Display()
{
	// Clear the view port and set the camera direction
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();

	cyclone::Vector3			pos			= sailboat.Position;
	cyclone::Vector3			offset			= {4.0f, 0, 0};
	offset = sailboat.getTransform().transformDirection(offset);
	gluLookAt(pos.x+offset.x, pos.y+5.0f, pos.z+offset.z,
	          pos.x, pos.y, pos.z,
	          0.0, 1.0, 0.0);

	glColor3f(0.6f,0.6f,0.6f);
	int bx = int(pos.x);
	int bz = int(pos.z);
	glBegin(GL_QUADS);
	for (int x = -20; x <= 20; x++) for (int z = -20; z <= 20; z++) {
		glVertex3f(bx+x-0.1f, 0, bz+z-0.1f);
		glVertex3f(bx+x-0.1f, 0, bz+z+0.1f);
		glVertex3f(bx+x+0.1f, 0, bz+z+0.1f);
		glVertex3f(bx+x+0.1f, 0, bz+z-0.1f);
	}
	glEnd();

	// Set the transform matrix for the aircraft
	cyclone::Matrix4 transform = sailboat.getTransform();
	GLfloat gl_transform[16];
	transform.fillGLArray(gl_transform);
	glPushMatrix();
	glMultMatrixf(gl_transform);

	// Draw the boat
	glColor3f(0,0,0);
	drawBoat();
	glPopMatrix();

	char buffer[256];
	sprintf_s(buffer, "Speed %.1f", sailboat.Velocity.magnitude() );
	glColor3f(0,0,0);
	RenderText(10.0f, 24.0f, buffer);

	sprintf_s(buffer, "Sail Control: %.1f", sail_control);
	RenderText(10.0f, 10.0f, buffer);
}

void SailboatDemo::Update() {
	float duration = (float)TimingData::get().lastFrameDuration * 0.001f;	// Find the duration of the last frame in seconds
	if (duration <= 0.0f) 
		return;

	sailboat.clearAccumulators();	// Start with no forces or acceleration.
	registry.UpdateForces(duration);	// Add the forces acting on the boat.
	sailboat.integrate(duration);	// Update the boat's physics.
	windspeed = windspeed * 0.9f + r.randomXZVector(1.0f);	// Change the wind speed.

	Application::Update();
}

const char* SailboatDemo::GetTitle() { return "Cyclone > Sail Boat Demo"; }

void SailboatDemo::Key(unsigned char key) {
	switch(key) {
	case 'q': case 'Q': sail_control -= 0.1f; break;
	case 'e': case 'E': sail_control += 0.1f; break;
	case 'w': case 'W': sail_control =  0.0f; break;
	default:
		Application::Key(key);
	}

	// Make sure the controls are in range
		 if (sail_control < -1.0f) sail_control = -1.0f;
	else if (sail_control >  1.0f) sail_control =  1.0f;

	//sail.setControl(sail_control);	// Update the control surfaces
}

Application* getApplication() { return new SailboatDemo(); }	// Called by the common demo framework to create an application object (with new) and return a pointer.