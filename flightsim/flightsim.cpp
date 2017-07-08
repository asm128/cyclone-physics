// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include "cyclone.h"
#include "ogl_headers.h"
#include "app.h"
#include "timing.h"

#include <stdio.h>
#include <cassert>

// The main demo class definition.
class FlightSimDemo : public Application{
	cyclone::AeroControl	Left_wing				;
	cyclone::AeroControl	Right_wing				;
	cyclone::AeroControl	Rudder					;
	cyclone::Aero			Tail					;
	cyclone::RigidBody		Aircraft				= {};
	cyclone::ForceRegistry	Registry				= {};
	
	cyclone::Vector3		Windspeed				= {};
	
	float					Left_wing_control		= 0;
	float					Right_wing_control		= 0;
	float					Rudder_control			= 0;
	
	void					ResetPlane				();

public:
	virtual					~FlightSimDemo			()						{}
							FlightSimDemo			();

	virtual const char*		GetTitle				();	// Returns the window title for the demo.
	virtual void			Display					();	// Display the particles. */
	virtual void			Update					();	// Update the particle positions. */
	virtual void			Key						(unsigned char key);	// Handle a key press. */

	virtual void			Mouse					(int, int, int, int)	{}	// Called when GLUT detects a mouse button press.
	virtual void			MouseDrag				(int, int)				{}	// Called when GLUT detects a mouse drag.
};

// Method definitions
FlightSimDemo::FlightSimDemo()
	: Application()
	, Right_wing	( cyclone::Matrix3(0,0,0, -1,-0.5f,0, 0,0,0)
					, cyclone::Matrix3(0,0,0, -0.995f,-0.5f,0, 0,0,0)
					, cyclone::Matrix3(0,0,0, -1.005f,-0.5f,0, 0,0,0)
					, {-1.0f, 0.0f, 2.0f}
					, &Windspeed
					)
	, Left_wing		( cyclone::Matrix3(0,0,0, -1,-0.5f,0, 0,0,0)
					, cyclone::Matrix3(0,0,0, -0.995f,-0.5f,0, 0,0,0)
					, cyclone::Matrix3(0,0,0, -1.005f,-0.5f,0, 0,0,0)
					, {-1.0f, 0.0f, -2.0f}
					, &Windspeed
					)
	, Rudder		( cyclone::Matrix3(0,0,0, 0,0,0, 0,0,0)
					, cyclone::Matrix3(0,0,0, 0,0,0, 0.01f,0,0)
					, cyclone::Matrix3(0,0,0, 0,0,0, -0.01f,0,0)
					, {2.0f, 0.5f, 0}
					, &Windspeed
					)
	, Tail			(cyclone::Matrix3(0,0,0, -1,-0.5f,0, 0,0,-0.1f), {2.0f, 0, 0}, &Windspeed)
{
    // Set up the aircraft rigid body.
    ResetPlane();

    Aircraft.setMass(2.5f);
    cyclone::Matrix3 it;
	it.setBlockInertiaTensor({2,1,1}, 1);
    Aircraft.setInertiaTensor(it);

    Aircraft.setDamping(0.8f, 0.8f);

    Aircraft.Acceleration = ::cyclone::Vector3::GRAVITY;
    Aircraft.calculateDerivedData();

    Aircraft.setAwake();
    Aircraft.setCanSleep(false);

    Registry.add(&Aircraft, &Left_wing);
    Registry.add(&Aircraft, &Right_wing);
    Registry.add(&Aircraft, &Rudder);
    Registry.add(&Aircraft, &Tail);
}

void FlightSimDemo::ResetPlane()
{
	Aircraft.Position		= {};
	Aircraft.Orientation	= {1,0,0,0};

	Aircraft.Velocity		= {};
	Aircraft.Rotation		= {};
}

static void drawAircraft()
{
    // Fuselage
    glPushMatrix();
    glTranslatef(-0.5f, 0, 0);
    glScalef(2.0f, 0.8f, 1.0f);
    glutSolidCube(1.0f);
    glPopMatrix();

    // Rear Fuselage
    glPushMatrix();
    glTranslatef(1.0f, 0.15f, 0);
    glScalef(2.75f, 0.5f, 0.5f);
    glutSolidCube(1.0f);
    glPopMatrix();

    // Wings
    glPushMatrix();
    glTranslatef(0, 0.3f, 0);
    glScalef(0.8f, 0.1f, 6.0f);
    glutSolidCube(1.0f);
    glPopMatrix();

    // Rudder
    glPushMatrix();
    glTranslatef(2.0f, 0.775f, 0);
    glScalef(0.75f, 1.15f, 0.1f);
    glutSolidCube(1.0f);
    glPopMatrix();

    // Tail-plane
    glPushMatrix();
    glTranslatef(1.9f, 0, 0);
    glScalef(0.85f, 0.1f, 2.0f);
    glutSolidCube(1.0f);
    glPopMatrix();
}

void FlightSimDemo::Display()
{
    // Clear the view port and set the camera direction
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    cyclone::Vector3						pos			= Aircraft.Position;
	cyclone::Vector3						offset		= { 4.0f + Aircraft.Velocity.magnitude(), 0, 0 };
    offset								= Aircraft.getTransform().transformDirection(offset);
    gluLookAt(pos.x+offset.x, pos.y+5.0f, pos.z+offset.z,
              pos.x, pos.y, pos.z,
              0.0, 1.0, 0.0);

    glColor3f(0.6f,0.6f,0.6f);
    int bx = int(pos.x);
    int bz = int(pos.z);
    glBegin(GL_QUADS);
    for (int x = -20; x <= 20; x++) for (int z = -20; z <= 20; z++)
    {
        glVertex3f(bx+x-0.1f, 0, bz+z-0.1f);
        glVertex3f(bx+x-0.1f, 0, bz+z+0.1f);
        glVertex3f(bx+x+0.1f, 0, bz+z+0.1f);
        glVertex3f(bx+x+0.1f, 0, bz+z-0.1f);
    }
    glEnd();

    // Set the transform matrix for the aircraft
    cyclone::Matrix4 transform = Aircraft.getTransform();
    GLfloat gl_transform[16];
    transform.fillGLArray(gl_transform);
    glPushMatrix();
    glMultMatrixf(gl_transform);

    // Draw the aircraft
    glColor3f(0,0,0);
    drawAircraft();
    glPopMatrix();

    glColor3f(0.8f, 0.8f, 0.8f);
    glPushMatrix();
    glTranslatef(0, -1.0f - pos.y, 0);
    glScalef(1.0f, 0.001f, 1.0f);
    glMultMatrixf(gl_transform);
    drawAircraft();
    glPopMatrix();

    char buffer[256];
    sprintf_s(
        buffer,
        "Altitude: %.1f | Speed %.1f",
        Aircraft.Position.y,
        Aircraft.Velocity.magnitude()
        );
    glColor3f(0,0,0);
    RenderText(10.0f, 24.0f, buffer);

    sprintf_s(
        buffer,
        "Left Wing: %.1f | Right Wing: %.1f | Rudder %.1f",
        Left_wing_control, Right_wing_control, Rudder_control
        );
    RenderText(10.0f, 10.0f, buffer);
}

void FlightSimDemo::Update()
{
    // Find the duration of the last frame in seconds
    float						duration						= (float)TimingData::get().lastFrameDuration * 0.001f;
    if (duration <= 0.0f) 
		return;

    Aircraft.clearAccumulators();	// Start with no forces or acceleration.

    // Add the propeller force
	cyclone::Vector3			propulsion						= {-10.0f, 0, 0};
    propulsion				= Aircraft.getTransform().transformDirection(propulsion);
    Aircraft.addForce		(propulsion);
    Registry.UpdateForces	(duration);	// Add the forces acting on the aircraft.
    Aircraft.integrate		(duration);	// Update the aircraft's physics.

    // Do a very basic collision detection and response with the ground.
    cyclone::Vector3 pos = Aircraft.Position;
    if (pos.y < 0.0f)
    {
        pos.y = 0.0f;
        Aircraft.Position = pos;

        if (Aircraft.Velocity.y < -10.0f)
            ResetPlane();
    }

    Application::Update();
}

const char* FlightSimDemo::GetTitle() { return "Cyclone > Flight Sim Demo"; }

void FlightSimDemo::Key(unsigned char key) {
    switch(key) {
    case 'q': case 'Q':	Rudder_control += 0.1f;															break;
    case 'e': case 'E': Rudder_control -= 0.1f;															break;
	case 'w': case 'W': Left_wing_control -= 0.1f;	Right_wing_control -= 0.1f;							break;
    case 's': case 'S': Left_wing_control += 0.1f;	Right_wing_control += 0.1f;							break;
    case 'd': case 'D': Left_wing_control -= 0.1f;	Right_wing_control += 0.1f;							break;
    case 'a': case 'A': Left_wing_control += 0.1f;	Right_wing_control -= 0.1f;							break;
    case 'x': case 'X': Left_wing_control  = 0.0f;	Right_wing_control  = 0.0f; Rudder_control = 0.0f;	break;
    case 'r': case 'R': ResetPlane();																	break;
    default:
        Application::Key(key);
    }

    // Make sure the controls are in range
		 if (Left_wing_control	< -1.0f) Left_wing_control	= -1.0f;
    else if (Left_wing_control	>  1.0f) Left_wing_control	= 1.0f;
		 if (Right_wing_control < -1.0f) Right_wing_control	= -1.0f;
    else if (Right_wing_control >  1.0f) Right_wing_control	= 1.0f;
		 if (Rudder_control		< -1.0f) Rudder_control		= -1.0f;
    else if (Rudder_control		>  1.0f) Rudder_control		= 1.0f;

    // Update the control surfaces
    Left_wing	.setControl(Left_wing_control	);
    Right_wing	.setControl(Right_wing_control	);
    Rudder		.setControl(Rudder_control		);
}

// Called by the common demo framework to create an application object (with new) and return a pointer.
Application* getApplication()
{
    return new FlightSimDemo();
}