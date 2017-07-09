// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include "cyclone.h"
#include "ogl_headers.h"
#include "app.h"
#include "timing.h"

#include <stdio.h>

enum ShotType
	{	UNUSED		= 0
	,	PISTOL
	,	ARTILLERY
	,	FIREBALL
	,	LASER
	};

class AmmoRound : public cyclone::CollisionSphere {
	cyclone::RigidBody					_ammoRoundBody;
public:
	ShotType							type;
	uint32_t							startTime;

									    AmmoRound						()							{ Body = &_ammoRoundBody; }
    // Draws the box, excluding its shadow. 
    void								render							()							{
        // Get the OpenGL transformation
        GLfloat mat[16];
        Body->getGLTransform(mat);

        glPushMatrix();
        glMultMatrixf(mat);
        glutSolidSphere(Radius, 20, 20);
        glPopMatrix();
    }

    // Sets the box to a specific location.
    void								setState						(ShotType shotType)			{
        type								= shotType;

        // Set the properties of the particle
        switch(type) {
        case PISTOL:
            Body->setMass(1.5f);
            Body->Velocity						= {0.0f, 0.0f, 20.0f};
            Body->Acceleration					= {0.0f, -0.5f, 0.0f};
            Body->setDamping(0.99f, 0.8f);
            Radius								= 0.2f;
            break;

        case ARTILLERY:
            Body->setMass(200.0f); // 200.0kg
            Body->Velocity						= {0.0f, 30.0f, 40.0f}; // 50m/s
            Body->Acceleration					= {0.0f, -21.0f, 0.0f};
            Body->setDamping(0.99f, 0.8f);
            Radius								= 0.4f;
            break;

        case FIREBALL:
            Body->setMass(4.0f); // 4.0kg - mostly blast damage
            Body->Velocity						= {0.0f, -0.5f, 10.0}; // 10m/s
            Body->Acceleration					= {0.0f,  0.3f, 0.0f}; // Floats up
            Body->setDamping(0.9f, 0.8f);
            Radius								= 0.6f;
            break;

        case LASER:
            // Note that this is the kind of laser bolt seen in films,
            // not a realistic laser beam!
            Body->setMass(0.1f); // 0.1kg - almost no weight
            Body->Velocity						= {0.0f, 0.0f, 100.0f	}; // 100m/s
            Body->Acceleration					= {0.0f, 0.0f, 0.0f		}; // No gravity
            Body->setDamping(0.99f, 0.8f);
            Radius								= 0.2f;
            break;
        }

        Body->setCanSleep(false);
        Body->setAwake();

        cyclone::Matrix3 tensor;
        double coeff = 0.4 * Body->getMass() * Radius * Radius;
        tensor.setInertiaTensorCoeffs(coeff,coeff,coeff);
        Body->setInertiaTensor(tensor);

        // Set the data common to all particle types
		Body->Position = {0.0f, 1.5f, 0.0f};
        startTime = TimingData::get().LastFrameTimestamp;

        // Clear the force accumulators
        Body->CalculateDerivedData();
        CalculateInternals();
    }
};

class Box : public cyclone::CollisionBox {
	cyclone::RigidBody						_boxBody;
public:
											Box					()						{ Body = &_boxBody; }

	// Draws the box, excluding its shadow.
	void									render				()						{
	    // Get the OpenGL transformation
	    GLfloat mat[16];
	    Body->getGLTransform(mat);

	    glPushMatrix();
	    glMultMatrixf(mat);
	    glScalef(HalfSize.x * 2, HalfSize.y * 2, HalfSize.z * 2);
	    glutSolidCube(1.0f);
	    glPopMatrix();
	}

	// Sets the box to a specific location.
	void									setState			(double z)					{
		Body->Position							= {0, 3, z};
		Body->setOrientation(1,0,0,0);
		Body->Velocity							= {};
		Body->Rotation							= {};
		HalfSize								= {1, 1, 1};

		double										mass				= HalfSize.x * HalfSize.y * HalfSize.z * 8.0f;
		Body->setMass(mass);

		::cyclone::Matrix3							tensor;
		tensor.setBlockInertiaTensor(HalfSize, mass);
		Body->setInertiaTensor(tensor);

		Body->LinearDamping						= 0.95f;
		Body->AngularDamping					= 0.8f;
		Body->clearAccumulators	();
		Body->Acceleration						= {0, -10.0f, 0};

		Body->setCanSleep(false);
		Body->setAwake();

		Body->CalculateDerivedData();
		CalculateInternals();
	}
};


// The main demo class definition.
class BigBallisticDemo : public RigidBodyApplication {
	const static uint32_t	AmmoRounds						= 256;							// Holds the maximum number of  rounds that can be fired.
	const static uint32_t	Boxes							= 2;							// Holds the number of boxes in the simulation.

	AmmoRound				Ammo			[AmmoRounds]	= {};							// Holds the particle data.
	Box						BoxData			[Boxes]			= {};							// Holds the box data. 
	ShotType				CurrentShotType					= {};							// Holds the current shot type. 

	virtual void			Reset							();								// Resets the position of all the boxes and primes the explosion. 
	virtual void			GenerateContacts				();								// Build the contacts for the current situation. 
	virtual void			UpdateObjects					(double duration);		// Processes the objects in the simulation forward in time. 
	void					Fire							();								// Dispatches a round. 

public:
							BigBallisticDemo				();		// Creates a new demo object. 

	virtual const char*		GetTitle						()										{ return "Cyclone > Big Ballistic Demo"; }	// Returns the window title for the demo. 
	virtual void			InitGraphics					();																					// Sets up the rendering.
	virtual void			Display							();																					// Display world.
	virtual void			Mouse							(int button, int state, int x, int y);												// Handle a mouse click.
	virtual void			Key								(unsigned char key);																// Handle a keypress.
};

// Method definitions
BigBallisticDemo::BigBallisticDemo()
	: RigidBodyApplication	()
	, CurrentShotType		(LASER)
{
	PauseSimulation		= false;
	Reset();
}


void BigBallisticDemo::InitGraphics()
{
    GLfloat lightAmbient[] = {0.8f,0.8f,0.8f,1.0f};
    GLfloat lightDiffuse[] = {0.9f,0.95f,1.0f,1.0f};

    glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse);

    glEnable(GL_LIGHT0);

    Application::InitGraphics();
}

void BigBallisticDemo::Reset(){
	for (AmmoRound *shot = Ammo; shot < Ammo + AmmoRounds; ++shot)	// Make all shots unused
		shot->type = UNUSED;

	double z = 20.0f;	// Initialise the box
	for (Box *box = BoxData; box < BoxData + Boxes; ++box) {
		box->setState(z);
		z += 90.0f;
	}
}

void BigBallisticDemo::Fire()
{
	AmmoRound				* shot						= 0;	// Find the first available round.
	for (shot = Ammo; shot < Ammo + AmmoRounds; ++shot)
		if (shot->type == UNUSED) 
			break;

	if (shot >= Ammo + AmmoRounds)		// If we didn't find a round, then exit - we can't fire.
		return;

	shot->setState(CurrentShotType);	// Set the shot
}

void BigBallisticDemo::UpdateObjects(double duration) {
	for(AmmoRound *shot = Ammo; shot < Ammo + AmmoRounds; shot++) {	// Update the physics of each particle in turn
		if (shot->type != UNUSED) {
			shot->Body->integrate(duration);	// Run the physics
			shot->CalculateInternals();
	
			// Check if the particle is now invalid
			if (shot->Body->Position.y < 0.0f ||
				shot->startTime+5000 < TimingData::get().LastFrameTimestamp ||
				shot->Body->Position.z > 200.0f)
			{
				shot->type = UNUSED;	// We simply set the shot type to be unused, so the memory it occupies can be reused by another shot.
			}
		}
	}
	
	
	for (Box *box = BoxData; box < BoxData + Boxes; box++) {	// Update the boxes
		box->Body->integrate(duration);	// Run the physics
		box->CalculateInternals();
	}
}

void BigBallisticDemo::Display()
{
	const static GLfloat lightPosition[] = {-1,1,0,0};

	// Clear the viewport and set the camera direction
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();
	gluLookAt(-25.0, 8.0, 5.0,  0.0, 5.0, 22.0,  0.0, 1.0, 0.0);

	// Draw a sphere at the firing point, and add a shadow projected onto the ground plane.
	glColor3f(0.0f, 0.0f, 0.0f);
	glPushMatrix();
	glTranslatef(0.0f, 1.5f, 0.0f);
	glutSolidSphere(0.1f, 5, 5);
	glTranslatef(0.0f, -1.5f, 0.0f);
	glColor3f(0.75f, 0.75f, 0.75f);
	glScalef(1.0f, 0.1f, 1.0f);
	glutSolidSphere(0.1f, 5, 5);
	glPopMatrix();

	// Draw some scale lines
	glColor3f(0.75f, 0.75f, 0.75f);
	glBegin(GL_LINES);
	for (uint32_t i = 0; i < 200; i += 10) {
		glVertex3f(-5.0f, 0.0f, i);
		glVertex3f(5.0f, 0.0f, i);
	}
	glEnd();

	// Render each particle in turn
	glColor3f(1,0,0);
	for (AmmoRound *shot = Ammo; shot < Ammo + AmmoRounds; shot++) 
		if (shot->type != UNUSED)
			shot->render();

	// Render the box
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
	glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
	glEnable(GL_COLOR_MATERIAL);
	glColor3f(1,0,0);
	for (Box *box = BoxData; box < BoxData + Boxes; box++)
		box->render();

	glDisable(GL_COLOR_MATERIAL);
	glDisable(GL_LIGHTING);
	glDisable(GL_DEPTH_TEST);

	// Render the description
	glColor3f(0.0f, 0.0f, 0.0f);
	RenderText(10.0f, 34.0f, "Click: Fire\n1-4: Select Ammo");

	// Render the name of the current shot type
	switch(CurrentShotType) {
	case PISTOL		: RenderText(10.0f, 10.0f, "Current Ammo: Pistol"	); break;
	case ARTILLERY	: RenderText(10.0f, 10.0f, "Current Ammo: Artillery"); break;
	case FIREBALL	: RenderText(10.0f, 10.0f, "Current Ammo: Fireball"	); break;
	case LASER		: RenderText(10.0f, 10.0f, "Current Ammo: Laser"	); break;
	}
}

void BigBallisticDemo::GenerateContacts()
{
	// Create the ground plane data
	cyclone::CollisionPlane					plane;
	plane.Direction						= {0,1,0};
	plane.Offset						= 0;

	// Set up the collision data structure
	Collisions.Reset(MaxContacts);
	Collisions.Friction					= 0.9;
	Collisions.Restitution				= 0.1;
	Collisions.Tolerance				= 0.1;

	// Check ground plane collisions
	for (Box *box = BoxData; box < BoxData + Boxes; box++) {
		if (!Collisions.HasMoreContacts()) 
			return;
		cyclone::CollisionDetector::boxAndHalfSpace(*box, plane, &Collisions);


		// Check for collisions with each shot
		for (AmmoRound *shot = Ammo; shot < Ammo + AmmoRounds; shot++) {
			if (shot->type != UNUSED) {
				if (!Collisions.HasMoreContacts()) 
					return;

				// When we get a collision, remove the shot
				if (cyclone::CollisionDetector::boxAndSphere(*box, *shot, &Collisions))
					shot->type = UNUSED;
			}
		}
	}

	// NB We aren't checking box-box collisions.
}

void BigBallisticDemo::Mouse(int button, int state, int x, int y) {
    if (state == GLUT_DOWN) 
		Fire();	// Fire the current weapon.
}

void BigBallisticDemo::Key(unsigned char key) {
    switch(key) {
    case '1': CurrentShotType = PISTOL		; break;
    case '2': CurrentShotType = ARTILLERY	; break;
    case '3': CurrentShotType = FIREBALL	; break;
    case '4': CurrentShotType = LASER		; break;
    case 'r': case 'R': 
		Reset(); 
		break;
    }
}


Application* getApplication() { return new BigBallisticDemo(); }	// Called by the common demo framework to create an application object (with new) and return a pointer.