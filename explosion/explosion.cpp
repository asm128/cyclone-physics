// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include "cyclone.h"
#include "ogl_headers.h"
#include "app.h"
#include "timing.h"

#include <stdio.h>

#define OBJECTS 5

// Holds a transform matrix for rendering objects
// reflected in the floor.
GLfloat floorMirror[16] =
	{ 1,  0, 0, 0
	, 0, -1, 0, 0
	, 0,  0, 1, 0
	, 0,  0, 0, 1
	};

class Ball : public cyclone::CollisionSphere
{
	::cyclone::RigidBody		_ballBody				= {};
public:
	inline						Ball					()					{ Body = &_ballBody; }

    // Draws the box, excluding its shadow. 
    void						render					()					{
		GLfloat mat[16];
		Body->getGLTransform(mat);	// 

        if (Body->IsAwake) 
			glColor3f(1.0f,0.7f,0.7f);
        else 
			glColor3f(0.7f,0.7f,1.0f);

        glPushMatrix();
        glMultMatrixf(mat);
        glutSolidSphere(radius, 20, 20);
        glPopMatrix();
    }

    // Draws the ground plane shadow for the box. 
    void						renderShadow			()					{
        // Get the OpenGL transformation
        GLfloat mat[16];
        Body->getGLTransform(mat);

        glPushMatrix();
        glScalef(1.0f, 0, 1.0f);
        glMultMatrixf(mat);
        glutSolidSphere(radius, 20, 20);
        glPopMatrix();
    }
	
	void setState	// Sets the box to a specific location.
		(	cyclone::Vector3		position
		,	cyclone::Quaternion		orientation
		,	cyclone::real			radius
		,	cyclone::Vector3		velocity
		)
    {
        Body->Position					= position;
        Body->Orientation				= orientation;
        Body->Velocity					= velocity;
        Body->Rotation					= cyclone::Vector3(0,0,0);
        Ball::radius					= radius;

        cyclone::real						mass					= 4.0f*0.3333f*3.1415f * radius*radius*radius;
        Body->setMass(mass);

        cyclone::Matrix3 tensor;
        cyclone::real coeff = 0.4f*mass*radius*radius;
        tensor.setInertiaTensorCoeffs(coeff,coeff,coeff);
        Body->setInertiaTensor(tensor);

        Body->LinearDamping				= 0.95f;
        Body->AngularDamping			= 0.8f;
        Body->clearAccumulators	();
        Body->setAcceleration	(0,-10.0f,0);

        //body->setCanSleep(false);
        Body->setAwake();

        Body->calculateDerivedData();
    }

    // Positions the box at a random location.
	void random(cyclone::Random *random) {
        const static cyclone::Vector3	minPos		(-5, 5, -5);
        const static cyclone::Vector3	maxPos		(5, 10, 5);
        cyclone::Random					r;
        setState
            ( random->randomVector		(minPos, maxPos)
            , random->randomQuaternion	()
            , random->randomReal		(0.5f, 1.5f)
            , cyclone::Vector3			()
            );
    }
};

class Box : public cyclone::CollisionBox
{
	::cyclone::RigidBody		_boxBody				= {};
public:
	bool isOverlapping;
	
	inline						Box						()			{ Body = &_boxBody; }

	// Draws the box, excluding its shadow.
	void						render					()			{
		GLfloat mat[16];
		Body->getGLTransform(mat);
	
			 if (isOverlapping) glColor3f(0.7f,1.0f,0.7f);
		else if (Body->IsAwake) glColor3f(1.0f,0.7f,0.7f);
		else					glColor3f(0.7f,0.7f,1.0f);
		
		glPushMatrix();
		glMultMatrixf(mat);
		glScalef(halfSize.x*2, halfSize.y*2, halfSize.z*2);
		glutSolidCube(1.0f);
		glPopMatrix();
	}
	
	// Draws the ground plane shadow for the box. 
	void						renderShadow			()			{
		GLfloat mat[16];
		Body->getGLTransform(mat);
		
		glPushMatrix();
		glScalef(1.0f, 0, 1.0f);
		glMultMatrixf(mat);
		glScalef(halfSize.x*2, halfSize.y*2, halfSize.z*2);
		glutSolidCube(1.0f);
		glPopMatrix();
	}
	
	// Sets the box to a specific location.
	void setState
		( const cyclone::Vector3	& position
		, const cyclone::Quaternion & orientation
		, const cyclone::Vector3	& extents
		, const cyclone::Vector3	& velocity
		)
	{
		Body->Position					= position;
		Body->setOrientation(orientation);
		Body->Velocity					= velocity;
		Body->Rotation					= {};
		halfSize						= extents;
		
		cyclone::real						mass					= halfSize.x * halfSize.y * halfSize.z * 8.0f;
		Body->setMass(mass);
		
		cyclone::Matrix3					tensor;
		tensor.setBlockInertiaTensor(halfSize, mass);
		Body->setInertiaTensor(tensor);
		Body->LinearDamping				= 0.95f;
		Body->AngularDamping			= 0.8f;
		Body->clearAccumulators		();
		Body->setAcceleration		(0,-10.0f,0);
		Body->setAwake				();
		Body->calculateDerivedData	();
	}
	
	// Positions the box at a random location.
	void random(cyclone::Random *random){
		const static cyclone::Vector3 minPos	= {-5,  5,-5};
		const static cyclone::Vector3 maxPos	= { 5, 10, 5};
		const static cyclone::Vector3 minSize	= {0.5f, 0.5f, 0.5f};
		const static cyclone::Vector3 maxSize	= {4.5f, 1.5f, 1.5f};
		
		setState
			( random->randomVector		(minPos, maxPos)
			, random->randomQuaternion	()
			, random->randomVector		(minSize, maxSize)
			, cyclone::Vector3			()
			);
	}
};

// The main demo class definition.
class ExplosionDemo : public RigidBodyApplication
{
	bool					EditMode, UpMode;
	const static uint32_t	Boxes				= OBJECTS;	// Holds the number of boxes in the simulation.
	Box						BoxData[Boxes];					// Holds the box data.
	const static uint32_t	Balls				= OBJECTS;	// Holds the number of balls in the simulation.
	Ball					BallData[Balls];				// Holds the ball data. 
	
	void					Fire				();	// Detonates the explosion. 
	virtual void			Reset				();	// Resets the position of all the boxes and primes the explosion. 
	virtual void			GenerateContacts	();	// Processes the contact generation code. 
	virtual void			UpdateObjects		(cyclone::real duration);	// Processes the objects in the simulation forward in time. 
public:
							ExplosionDemo		();	// Creates a new demo object. 
	virtual void			InitGraphics		();	// Sets up the rendering. 
	virtual const char*		GetTitle			()												{ return "Cyclone > Explosion Demo"; }
	virtual void			Display				();	// Display the particle positions. 
	virtual void			Key					(unsigned char key);	// Handles a key press.
	virtual void			MouseDrag			(int x, int y);			// Handle a mouse drag.
};

// Method definitions
ExplosionDemo::ExplosionDemo()
	: RigidBodyApplication	()
	, EditMode				(false)
	, UpMode				(false)
{
	Reset();	// Reset the position of the boxes
}

void ExplosionDemo::Fire()
{
    cyclone::Vector3 pos = BallData[0].Body->Position;
    pos.normalise();

    BallData[0].Body->addForce(pos * -1000.0f);
}

void ExplosionDemo::Reset()
{
    Box *box = BoxData;

    box++->setState(cyclone::Vector3(0,3,0),
                    cyclone::Quaternion(),
                    cyclone::Vector3(4,1,1),
                    cyclone::Vector3(0,1,0));

    if (Boxes > 1) {
        box++->setState(cyclone::Vector3(0,4.75,2),
                        cyclone::Quaternion(1.0,0.1,0.05,0.01),
                        cyclone::Vector3(1,1,4),
                        cyclone::Vector3(0,1,0));
    }

    // Create the random objects
    cyclone::Random random;
    for (; box < BoxData + Boxes; box++)
        box->random(&random);

    for (Ball *ball = BallData; ball < BallData + Balls; ball++)
        ball->random(&random);

    // Reset the contacts
    CData.contactCount = 0;
}

void ExplosionDemo::GenerateContacts() {
    // Note that this method makes a lot of use of early returns to avoid
    // processing lots of potential contacts that it hasn't got room to
    // store.

    // Create the ground plane data
    cyclone::CollisionPlane		plane;
    plane.direction			= cyclone::Vector3(0,1,0);
    plane.offset			= 0;

    // Set up the collision data structure
    CData.reset(MaxContacts);
    CData.friction			= (cyclone::real)0.9;
    CData.restitution		= (cyclone::real)0.6;
    CData.tolerance			= (cyclone::real)0.1;

    // Perform exhaustive collision detection
    cyclone::Matrix4 transform, otherTransform;
    cyclone::Vector3 position, otherPosition;
    for (Box *box = BoxData; box < BoxData + Boxes; box++) {
        // Check for collisions with the ground plane
        if (!CData.hasMoreContacts()) 
			return;
        cyclone::CollisionDetector::boxAndHalfSpace(*box, plane, &CData);

        // Check for collisions with each other box
        for (Box *other = box+1; other < BoxData + Boxes; other++) {
            if (!CData.hasMoreContacts()) 
				return;
            cyclone::CollisionDetector::boxAndBox(*box, *other, &CData);

            if (cyclone::IntersectionTests::boxAndBox(*box, *other))
                box->isOverlapping = other->isOverlapping = true;
        }

        // Check for collisions with each ball
        for (Ball *other = BallData; other < BallData + Balls; other++) {
            if (!CData.hasMoreContacts()) 
				return;
            cyclone::CollisionDetector::boxAndSphere(*box, *other, &CData);
        }
    }

    for (Ball *ball = BallData; ball < BallData + Balls; ball++) {
        // Check for collisions with the ground plane
        if (!CData.hasMoreContacts()) 
			return;
        cyclone::CollisionDetector::sphereAndHalfSpace(*ball, plane, &CData);

        for (Ball *other = ball + 1; other < BallData + Balls; other++) {
            // Check for collisions with the ground plane
            if (!CData.hasMoreContacts()) 
				return;
            cyclone::CollisionDetector::sphereAndSphere(*ball, *other, &CData);
        }
    }
}

void ExplosionDemo::UpdateObjects(cyclone::real duration) {
	for (Box *box = BoxData; box < BoxData + Boxes; box++) {	// Update the physics of each box in turn
		box->Body->integrate(duration);	// Run the physics
		box->CalculateInternals();
		box->isOverlapping = false;
	}
	
	for (Ball *ball = BallData; ball < BallData + Balls; ball++) {	// Update the physics of each ball in turn
		ball->Body->integrate(duration);	// Run the physics
		ball->CalculateInternals();
	}
}


void ExplosionDemo::InitGraphics()
{
    GLfloat lightAmbient[] = {0.8f,0.8f,0.8f,1.0f};
    GLfloat lightDiffuse[] = {0.9f,0.95f,1.0f,1.0f};

    glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse);

    glEnable(GL_LIGHT0);

    Application::InitGraphics();
}

void ExplosionDemo::Display()
{
    const static GLfloat		lightPosition		[] = {1,-1,0,0};
    const static GLfloat		lightPositionMirror	[] = {1, 1,0,0};

    for (Box *box = BoxData; box < BoxData + Boxes; box++) {	// Update the transform matrices of each box in turn
        box->CalculateInternals();
        box->isOverlapping		= false;
    }

    // Update the transform matrices of each ball in turn
    for (Ball *ball = BallData; ball < BallData + Balls; ball++)
        ball->CalculateInternals();	// Run the physics

    
    RigidBodyApplication::Display();	// Clear the viewport and set the camera direction

    // Render each element in turn as a shadow
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
    glPushMatrix();
    glMultMatrixf(floorMirror);
    glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
    glEnable(GL_COLOR_MATERIAL);
    for (Box *box = BoxData; box < BoxData + Boxes; box++)
        box->render();

    for (Ball *ball = BallData; ball < BallData + Balls; ball++)
        ball->render();

    glPopMatrix();
    glDisable(GL_LIGHTING);
    glDisable(GL_COLOR_MATERIAL);

    // Draw some scale circles
    glColor3f(0.75, 0.75, 0.75);
    for (unsigned i = 1; i < 20; i++)
    {
        glBegin(GL_LINE_LOOP);
        for (unsigned j = 0; j < 32; j++)
        {
            float theta = 3.1415926f * j / 16.0f;
            glVertex3f(i*cosf(theta),0.0f,i*sinf(theta));
        }
        glEnd();
    }
    glBegin(GL_LINES);
    glVertex3f(-20,0,0);
    glVertex3f(20,0,0);
    glVertex3f(0,0,-20);
    glVertex3f(0,0,20);
    glEnd();

	// Render each shadow in turn
	glEnable(GL_BLEND);
	glColor4f(0,0,0,0.1f);
	glDisable(GL_DEPTH_TEST);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	for (Box *box = BoxData; box < BoxData + Boxes; box++)
		box->renderShadow();

	for (Ball *ball = BallData; ball < BallData + Balls; ball++)
		ball->renderShadow();

	glDisable(GL_BLEND);

    // Render the boxes themselves
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glLightfv(GL_LIGHT0, GL_POSITION, lightPositionMirror);
    glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
    glEnable(GL_COLOR_MATERIAL);
    for (Box *box = BoxData; box < BoxData + Boxes; box++)
        box->render();

	for (Ball *ball = BallData; ball < BallData + Balls; ball++)
        ball->render();

	glDisable(GL_COLOR_MATERIAL);
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);

    // Finish the frame, rendering any additional information
    DrawDebug();
}

void ExplosionDemo::MouseDrag(int x, int y)
{
    if (EditMode) {
        BoxData[0].Body->Position = BoxData[0].Body->Position + cyclone::Vector3((x-Last_x) * 0.125f, 0, (y-Last_y) * 0.125f);
        BoxData[0].Body->calculateDerivedData();
    }
    else if (UpMode) {
        BoxData[0].Body->Position = BoxData[0].Body->Position + cyclone::Vector3(0, (y-Last_y) * 0.125f, 0);
        BoxData[0].Body->calculateDerivedData();
    }
    else {
        RigidBodyApplication::MouseDrag(x, y);
    }

    // Remember the position
    Last_x = x;
    Last_y = y;
}


void ExplosionDemo::Key(unsigned char key) {
	switch(key) {
	case 'e': case 'E': EditMode	= !EditMode	; UpMode	= false; return;
	case 't': case 'T': UpMode		= !UpMode	; EditMode	= false; return;
	case 'w': case 'W':
		for (Box *box	= BoxData	; box	< BoxData	+ Boxes; ++box	) box	->Body->setAwake();
		for (Ball *ball = BallData	; ball	< BallData	+ Balls; ++ball	) ball	->Body->setAwake();
		return;
	}

	RigidBodyApplication::Key(key);
}

// Called by the common demo framework to create an application object (with new) and return a pointer.
Application* getApplication()
{
    return new ExplosionDemo();
}