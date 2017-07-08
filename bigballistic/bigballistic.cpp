// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include "cyclone.h"
#include "ogl_headers.h"
#include "app.h"
#include "timing.h"

#include <stdio.h>

enum ShotType
{
    UNUSED = 0,
    PISTOL,
    ARTILLERY,
    FIREBALL,
    LASER
};

class AmmoRound : public cyclone::CollisionSphere
{
public:
    ShotType type;
    unsigned startTime;

    AmmoRound()
    {
        Body = new cyclone::RigidBody;
    }

    ~AmmoRound()
    {
        delete Body;
    }

    /** Draws the box, excluding its shadow. */
    void render()
    {
        // Get the OpenGL transformation
        GLfloat mat[16];
        Body->getGLTransform(mat);

        glPushMatrix();
        glMultMatrixf(mat);
        glutSolidSphere(radius, 20, 20);
        glPopMatrix();
    }

    /** Sets the box to a specific location. */
    void setState(ShotType shotType)
    {
        type = shotType;

        // Set the properties of the particle
        switch(type)
        {
        case PISTOL:
            Body->setMass(1.5f);
            Body->setVelocity(0.0f, 0.0f, 20.0f);
            Body->setAcceleration(0.0f, -0.5f, 0.0f);
            Body->setDamping(0.99f, 0.8f);
            radius = 0.2f;
            break;

        case ARTILLERY:
            Body->setMass(200.0f); // 200.0kg
            Body->setVelocity(0.0f, 30.0f, 40.0f); // 50m/s
            Body->setAcceleration(0.0f, -21.0f, 0.0f);
            Body->setDamping(0.99f, 0.8f);
            radius = 0.4f;
            break;

        case FIREBALL:
            Body->setMass(4.0f); // 4.0kg - mostly blast damage
            Body->setVelocity(0.0f, -0.5f, 10.0); // 10m/s
            Body->setAcceleration(0.0f, 0.3f, 0.0f); // Floats up
            Body->setDamping(0.9f, 0.8f);
            radius = 0.6f;
            break;

        case LASER:
            // Note that this is the kind of laser bolt seen in films,
            // not a realistic laser beam!
            Body->setMass(0.1f); // 0.1kg - almost no weight
            Body->setVelocity(0.0f, 0.0f, 100.0f); // 100m/s
            Body->setAcceleration(0.0f, 0.0f, 0.0f); // No gravity
            Body->setDamping(0.99f, 0.8f);
            radius = 0.2f;
            break;
        }

        Body->setCanSleep(false);
        Body->setAwake();

        cyclone::Matrix3 tensor;
        cyclone::real coeff = 0.4f * Body->getMass()*radius*radius;
        tensor.setInertiaTensorCoeffs(coeff,coeff,coeff);
        Body->setInertiaTensor(tensor);

        // Set the data common to all particle types
		Body->Position = {0.0f, 1.5f, 0.0f};
        startTime = TimingData::get().lastFrameTimestamp;

        // Clear the force accumulators
        Body->calculateDerivedData();
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
	    glScalef(halfSize.x*2, halfSize.y*2, halfSize.z*2);
	    glutSolidCube(1.0f);
	    glPopMatrix();
	}

	// Sets the box to a specific location.
	void									setState			(::cyclone::real z)		{
		Body->Position					= {0, 3, z};
		Body->setOrientation(1,0,0,0);
		Body->Velocity					= {};
		Body->Rotation					= ::cyclone::Vector3(0,0,0);
		halfSize						= ::cyclone::Vector3(1,1,1);

		::cyclone::real mass = halfSize.x * halfSize.y * halfSize.z * 8.0f;
		Body->setMass(mass);

		::cyclone::Matrix3 tensor;
		tensor.setBlockInertiaTensor(halfSize, mass);
		Body->setInertiaTensor(tensor);

		Body->LinearDamping		= 0.95f;
		Body->AngularDamping	= 0.8f;
		Body->clearAccumulators	();
		Body->setAcceleration	(0,-10.0f,0);

		Body->setCanSleep(false);
		Body->setAwake();

		Body->calculateDerivedData();
		CalculateInternals();
	}
};


// The main demo class definition.
class BigBallisticDemo : public RigidBodyApplication {
	const static unsigned	ammoRounds						= 256;							// Holds the maximum number of  rounds that can be fired.
	const static unsigned	boxes							= 2;							// Holds the number of boxes in the simulation.

	AmmoRound				ammo			[ammoRounds]	= {};							// Holds the particle data.
	Box						boxData			[boxes]			= {};							// Holds the box data. 
	ShotType				currentShotType					= {};							// Holds the current shot type. 

	virtual void			reset							();								// Resets the position of all the boxes and primes the explosion. 
	virtual void			GenerateContacts				();								// Build the contacts for the current situation. 
	virtual void			updateObjects					(cyclone::real duration);		// Processes the objects in the simulation forward in time. 
	void					fire							();								// Dispatches a round. 

public:
							BigBallisticDemo				();		// Creates a new demo object. 

	virtual const char*		getTitle						()										{ return "Cyclone > Big Ballistic Demo"; }	// Returns the window title for the demo. 
	virtual void			initGraphics					();																					// Sets up the rendering.
	virtual void			display							();																					// Display world.
	virtual void			mouse							(int button, int state, int x, int y);												// Handle a mouse click.
	virtual void			key								(unsigned char key);																// Handle a keypress.
};

// Method definitions
BigBallisticDemo::BigBallisticDemo()
	: RigidBodyApplication	()
	, currentShotType		(LASER)
{
	pauseSimulation		= false;
	reset();
}


void BigBallisticDemo::initGraphics()
{
    GLfloat lightAmbient[] = {0.8f,0.8f,0.8f,1.0f};
    GLfloat lightDiffuse[] = {0.9f,0.95f,1.0f,1.0f};

    glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse);

    glEnable(GL_LIGHT0);

    Application::initGraphics();
}

void BigBallisticDemo::reset(){
	for (AmmoRound *shot = ammo; shot < ammo + ammoRounds; ++shot)	// Make all shots unused
		shot->type = UNUSED;

	cyclone::real z = 20.0f;	// Initialise the box
	for (Box *box = boxData; box < boxData + boxes; ++box) {
		box->setState(z);
		z += 90.0f;
	}
}

void BigBallisticDemo::fire()
{
	AmmoRound				* shot;	// Find the first available round.
	for (shot = ammo; shot < ammo + ammoRounds; ++shot)
		if (shot->type == UNUSED) 
			break;

	if (shot >= ammo + ammoRounds)		// If we didn't find a round, then exit - we can't fire.
		return;

	shot->setState(currentShotType);	// Set the shot
}

void BigBallisticDemo::updateObjects(cyclone::real duration) {
	for(AmmoRound *shot = ammo; shot < ammo+ammoRounds; shot++) {	// Update the physics of each particle in turn
		if (shot->type != UNUSED) {
			shot->Body->integrate(duration);	// Run the physics
			shot->CalculateInternals();
	
			// Check if the particle is now invalid
			if (shot->Body->Position.y < 0.0f ||
				shot->startTime+5000 < TimingData::get().lastFrameTimestamp ||
				shot->Body->Position.z > 200.0f)
			{
				shot->type = UNUSED;	// We simply set the shot type to be unused, so the memory it occupies can be reused by another shot.
			}
		}
	}
	
	
	for (Box *box = boxData; box < boxData+boxes; box++) {	// Update the boxes
		box->Body->integrate(duration);	// Run the physics
		box->CalculateInternals();
	}
}

void BigBallisticDemo::display()
{
	const static GLfloat lightPosition[] = {-1,1,0,0};

	// Clear the viewport and set the camera direction
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();
	gluLookAt(-25.0, 8.0, 5.0,  0.0, 5.0, 22.0,  0.0, 1.0, 0.0);

	// Draw a sphere at the firing point, and add a shadow projected
	// onto the ground plane.
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
	for (unsigned i = 0; i < 200; i += 10) {
		glVertex3f(-5.0f, 0.0f, i);
		glVertex3f(5.0f, 0.0f, i);
	}
	glEnd();

	// Render each particle in turn
	glColor3f(1,0,0);
	for (AmmoRound *shot = ammo; shot < ammo+ammoRounds; shot++) 
		if (shot->type != UNUSED)
			shot->render();

	// Render the box
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
	glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
	glEnable(GL_COLOR_MATERIAL);
	glColor3f(1,0,0);
	for (Box *box = boxData; box < boxData+boxes; box++)
		box->render();

	glDisable(GL_COLOR_MATERIAL);
	glDisable(GL_LIGHTING);
	glDisable(GL_DEPTH_TEST);

	// Render the description
	glColor3f(0.0f, 0.0f, 0.0f);
	renderText(10.0f, 34.0f, "Click: Fire\n1-4: Select Ammo");

	// Render the name of the current shot type
	switch(currentShotType) {
	case PISTOL		: renderText(10.0f, 10.0f, "Current Ammo: Pistol"	); break;
	case ARTILLERY	: renderText(10.0f, 10.0f, "Current Ammo: Artillery"); break;
	case FIREBALL	: renderText(10.0f, 10.0f, "Current Ammo: Fireball"	); break;
	case LASER		: renderText(10.0f, 10.0f, "Current Ammo: Laser"	); break;
	}
}

void BigBallisticDemo::GenerateContacts()
{
    // Create the ground plane data
    cyclone::CollisionPlane plane;
    plane.direction = cyclone::Vector3(0,1,0);
    plane.offset = 0;

    // Set up the collision data structure
    cData.reset(maxContacts);
    cData.friction = (cyclone::real)0.9;
    cData.restitution = (cyclone::real)0.1;
    cData.tolerance = (cyclone::real)0.1;

    // Check ground plane collisions
    for (Box *box = boxData; box < boxData+boxes; box++)
    {
        if (!cData.hasMoreContacts()) return;
        cyclone::CollisionDetector::boxAndHalfSpace(*box, plane, &cData);


        // Check for collisions with each shot
        for (AmmoRound *shot = ammo; shot < ammo+ammoRounds; shot++)
        {
            if (shot->type != UNUSED)
            {
                if (!cData.hasMoreContacts()) return;

                // When we get a collision, remove the shot
                if (cyclone::CollisionDetector::boxAndSphere(*box, *shot, &cData))
                {
                    shot->type = UNUSED;
                }
            }
        }
    }

    // NB We aren't checking box-box collisions.
}

void BigBallisticDemo::mouse(int button, int state, int x, int y)
{
    // Fire the current weapon.
    if (state == GLUT_DOWN) fire();
}

void BigBallisticDemo::key(unsigned char key)
{
    switch(key) {
    case '1': currentShotType = PISTOL		; break;
    case '2': currentShotType = ARTILLERY	; break;
    case '3': currentShotType = FIREBALL	; break;
    case '4': currentShotType = LASER		; break;
    case 'r': case 'R': 
		reset(); 
		break;
    }
}

/**
 * Called by the common demo framework to create an application
 * object (with new) and return a pointer.
 */
Application* getApplication()
{
    return new BigBallisticDemo();
}