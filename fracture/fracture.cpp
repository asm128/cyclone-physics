// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include "cyclone.h"
#include "ogl_headers.h"
#include "app.h"
#include "timing.h"

#include <stdio.h>

#define MAX_BLOCKS 9

cyclone::Random global_random;

class Block : public cyclone::CollisionBox {
	::cyclone::RigidBody		_blockBody;
public:
	bool						Exists;

								Block							()						: Exists	(false)				{ Body = &_blockBody; }
	
	// Draws the block.
	void						Render							()						{
        // Get the OpenGL transformation
        GLfloat mat[16];
        Body->getGLTransform(mat);

        if (Body->IsAwake)	glColor3f(1.0f,0.7f,0.7f);
        else				glColor3f(0.7f,0.7f,1.0f);

        glPushMatrix();
        glMultMatrixf(mat);
        glScalef(HalfSize.x*2, HalfSize.y*2, HalfSize.z*2);
        glutSolidCube(1.0f);
        glPopMatrix();
    }

    //// Sets the block to a specific location. 
    //void SetState(const cyclone::Vector3 &position,
    //              const cyclone::Quaternion &orientation,
    //              const cyclone::Vector3 &extents,
    //              const cyclone::Vector3 &velocity)
    //{
    //    Body->Position				= position;
    //    Body->setOrientation(orientation);
    //    Body->Velocity				= velocity;
	//	Body->Rotation				= {};
    //    HalfSize					= extents;
	//
    //    double							mass					= HalfSize.x * HalfSize.y * HalfSize.z * 8.0f;
    //    Body->setMass(mass);
	//
    //    cyclone::Matrix3				tensor;
    //    tensor.setBlockInertiaTensor(HalfSize, mass);
    //    Body->setInertiaTensor(tensor);
	//
    //    Body->LinearDamping			= 0.95f;
    //    Body->AngularDamping		= 0.8f;
    //    Body->clearAccumulators();
	//	Body->Acceleration			= {0, -10.0f, 0};
	//
    //    //body->setCanSleep(false);
    //    Body->setAwake();
	//
    //    Body->CalculateDerivedData();
    //}

	// Calculates and sets the mass and inertia tensor of this block, assuming it has the given constant density.
	void CalculateMassProperties(double invDensity) {
        // Check for infinite mass
        if (invDensity <= 0) {		// Just set zeros for both mass and inertia tensor
            Body->Mass.InverseMass = 0;
            Body->Mass.setInverseInertiaTensor(cyclone::Matrix3());
        }
        else {	// Otherwise we need to calculate the mass
            double	volume		= HalfSize.magnitude() * 2.0;
            double	mass		= volume / invDensity;

            Body->Mass.setMass(mass);

            // And calculate the inertia tensor from the mass and size
            mass *= 0.333f;
            cyclone::Matrix3 tensor;
            tensor.setInertiaTensorCoeffs
				( mass * HalfSize.y * HalfSize.y + HalfSize.z * HalfSize.z
                , mass * HalfSize.y * HalfSize.x + HalfSize.z * HalfSize.z
                , mass * HalfSize.y * HalfSize.x + HalfSize.z * HalfSize.y
                );
            Body->Mass.setInertiaTensor(tensor);
        }

    }

	// Performs the division of the given block into four, writing the eight new blocks into the given blocks array. 
	// The blocks array can be a pointer to the same location as the target pointer: since the original block is always deleted, this effectively reuses its storage.
	// The algorithm is structured to allow this reuse.
    void DivideBlock(const cyclone::Contact& contact,
        Block* target, Block* blocks)
    {
        // Find out if we're block one or two in the contact structure, and therefore what the contact normal is.
        cyclone::Vector3							normal									= contact.ContactNormal;
        cyclone::RigidBody							* body									= contact.Body[0];
        if (body != target->Body) {
            normal.invert();
            body = contact.Body[1];
        }

        // Work out where on the body (in body coordinates) the contact is and its direction.
        cyclone::Vector3							point									= body->GetPointInLocalSpace(contact.ContactPoint);
        normal									= body->getDirectionInLocalSpace(normal);
        point									= point - normal * (point * normal);	// Work out the centre of the split: this is the point coordinates for each of the axes perpendicular to the normal, and 0 for the axis along the normal.

        cyclone::Vector3							size									= target->HalfSize;	// Take a copy of the half size, so we can create the new blocks.

        // Take a copy also of the body's other data.
        cyclone::RigidBody							tempBody;
        tempBody.Pivot.Position					= body->Pivot.Position				;
        tempBody.Pivot.Orientation				= body->Pivot.Orientation			;
        tempBody.Force.Velocity					= body->Force.Velocity				;
        tempBody.Force.Rotation					= body->Force.Rotation				;
        tempBody.Mass.LinearDamping				= body->Mass.LinearDamping			;
        tempBody.Mass.AngularDamping			= body->Mass.AngularDamping			;
        tempBody.Mass.InverseInertiaTensor		= body->Mass.InverseInertiaTensor	;
        tempBody.CalculateDerivedData();
		
		target->Exists							= false;	// Remove the old block
        double								invDensity								= HalfSize.magnitude()*8 * body->Mass.InverseMass;	// Work out the inverse density of the old block

        // Now split the block into eight.
        for (uint32_t i = 0; i < 8; i++) {
            // Find the minimum and maximum extents of the new block in old-block coordinates
            cyclone::Vector3 min, max;
            if ((i & 1) == 0) {
                min.x = -size.x;
                max.x = point.x;
            } else {
                min.x = point.x;
                max.x = size.x;
            }
            if ((i & 2) == 0) {
                min.y = -size.y;
                max.y = point.y;
            } else {
                min.y = point.y;
                max.y = size.y;
            }
            if ((i & 4) == 0) {
                min.z = -size.z;
                max.z = point.z;
            } else {
                min.z = point.z;
                max.z = size.z;
            }

            // Get the origin and half size of the block, in old-body local coordinates.
            cyclone::Vector3 halfSize = (max - min) * 0.5f;
            cyclone::Vector3 newPos = halfSize + min;
            newPos = tempBody.getPointInWorldSpace(newPos);	// Convert the origin to world coordinates.

            cyclone::Vector3 direction = newPos - contact.ContactPoint;	// Work out the direction to the impact.
            direction.normalise();

            // Set the body's properties (we assume the block has a body already that we're going to overwrite).
            blocks[i].Body->Pivot.Position			= newPos;
            blocks[i].Body->Force.Velocity			= tempBody.Force.Velocity + direction * 10.0f;
            blocks[i].Body->Pivot.setOrientation	(tempBody.Pivot.Orientation);
            blocks[i].Body->Force.Rotation			= tempBody.Force.Rotation;
            blocks[i].Body->Mass.LinearDamping		= tempBody.Mass.LinearDamping	;
            blocks[i].Body->Mass.AngularDamping		= tempBody.Mass.AngularDamping	;
            blocks[i].Body->setAwake				(true);
            blocks[i].Body->Force.Acceleration		= (cyclone::Vector3::GRAVITY);
            blocks[i].Body->clearAccumulators		();
            blocks[i].Body->CalculateDerivedData	();
            blocks[i].Offset						= cyclone::Matrix4();
            blocks[i].Exists						= true;
            blocks[i].HalfSize						= halfSize;

            blocks[i].CalculateMassProperties(invDensity);	// Finally calculate the mass and inertia tensor of the new block
        }
    }
};

// The main demo class definition.
class FractureDemo : public RigidBodyApplication {
	// -- Tracks if a block has been hit.
	bool						Hit;
	bool						Ball_active;
	uint32_t					Fracture_contact;

	cyclone::Random				Random;								// Handle random numbers.
	Block						Blocks[MAX_BLOCKS];					// Holds the bodies. 
	cyclone::CollisionSphere	Ball;								// Holds the projectile. 

	virtual void				GenerateContacts					();					// Processes the contact generation code. 
	virtual void				UpdateObjects						(double duration);	// Processes the objects in the simulation forward in time. 
	virtual void				Reset								();					// Resets the position of all the blocks. 
	virtual void				Update								();					// Processes the physics. 

public:
								FractureDemo						();					// Creates a new demo object. 

	virtual void				Display								();					// Display the particle positions. 
	virtual const char*			GetTitle							()					{ return "Cyclone > Fracture Demo"; }	// Returns the window title for the demo. 
};

// Method definitions
								FractureDemo::FractureDemo			()
{
	// Create the ball.
	Ball.Body										= new cyclone::RigidBody();
	Ball.Radius										= 0.25f;
	Ball.Body->Mass.setMass(5.0f);
	Ball.Body->Mass.setDamping(0.9f, 0.9f);
	cyclone::Matrix3									it;
	it.setDiagonal(5.0f, 5.0f, 5.0f);
	Ball.Body->Mass.setInertiaTensor(it);
	Ball.Body->Force.Acceleration					= cyclone::Vector3::GRAVITY;
	
	Ball.Body->setCanSleep	(false);
	Ball.Body->setAwake		(true);

	Reset();	// Set up the initial block
}

void FractureDemo::GenerateContacts()
{
	Hit													= false;

	// Create the ground plane data
	cyclone::CollisionPlane									plane;
	plane.Direction										= {0,1,0};
	plane.Offset										= 0;

	// Set up the collision data structure
	Collisions.Reset(MaxContacts);
	Collisions.Friction									= (double)0.9;
	Collisions.Restitution								= (double)0.2;
	Collisions.Tolerance								= (double)0.1;

	// Perform collision detection
	cyclone::Matrix4									transform, otherTransform;
	cyclone::Vector3									position, otherPosition;
	for (Block *block = Blocks; block < Blocks + MAX_BLOCKS; ++block) {
		if (!block->Exists) 
			continue;

		// Check for collisions with the ground plane
		if (!Collisions.HasMoreContacts()) 
			return;

		cyclone::CollisionDetector::boxAndHalfSpace(*block, plane, &Collisions);

		if (Ball_active) {	// And with the sphere
			if (!Collisions.HasMoreContacts()) 
				return;
			if (cyclone::CollisionDetector::boxAndSphere(*block, Ball, &Collisions)) {
				Hit												= true;
				Fracture_contact								= Collisions.ContactCount-1;
			}
		}

		for (Block *other = block+1; other < Blocks + MAX_BLOCKS; other++) {	// Check for collisions with each other box
			if (!other->Exists) 
				continue;
			if (!Collisions.HasMoreContacts()) 
				return;
			cyclone::CollisionDetector::boxAndBox(*block, *other, &Collisions);
		}
	}

	if (Ball_active) {	// Check for sphere ground collisions
		if (!Collisions.HasMoreContacts()) 
			return;
		cyclone::CollisionDetector::sphereAndHalfSpace(Ball, plane, &Collisions);
	}
}

void FractureDemo::Reset()
{
    // Only the first block exists
    Blocks[0].Exists = true;
    for (Block *block = Blocks + 1; block < Blocks + MAX_BLOCKS; block++)
        block->Exists = false;

    // Set the first block
	Blocks[0].HalfSize					= {4,4,4};
	Blocks[0].Body->Pivot.Position		= {0, 7, 0};
	Blocks[0].Body->Pivot.Orientation	= {1,0,0,0};
    Blocks[0].Body->Force.Velocity		= {};
    Blocks[0].Body->Force.Rotation		= {};
    Blocks[0].Body->Mass.setMass(100.0f);
    cyclone::Matrix3						it;
    it.setBlockInertiaTensor(Blocks[0].HalfSize, 100.0f);
    Blocks[0].Body->Mass.setInertiaTensor(it);
    Blocks[0].Body->Mass.setDamping(0.9f, 0.9f);
    Blocks[0].Body->CalculateDerivedData();
    Blocks[0].CalculateInternals();

    Blocks[0].Body->Force.Acceleration	= cyclone::Vector3::GRAVITY;
    Blocks[0].Body->setAwake(true);
    Blocks[0].Body->setCanSleep(true);


    Ball_active = true;

    // Set up the ball
	Ball.Body->Pivot.Position			= {0,5.0f,20.0f};
	Ball.Body->Pivot.Orientation		= {1, 0, 0, 0};
    Ball.Body->Force.Velocity			= {
        Random.RandomBinomial(4.0f),
        Random.RandomReal(1.0f, 6.0f),
        -20.0f
	};
	Ball.Body->Force.Rotation			= {};
    Ball.Body->CalculateDerivedData();
    Ball.Body->setAwake(true);
    Ball.CalculateInternals();

    Hit = false;

    Collisions.ContactCount = 0;	// Reset the contacts
}

void FractureDemo::Update() {
	RigidBodyApplication::Update();
	if (Hit) {	// Handle fractures.
		Blocks[0].DivideBlock(
			Collisions.ContactArray[Fracture_contact],
			Blocks,
			Blocks+1
			);
		Ball_active = false;
	}
}

void FractureDemo::UpdateObjects(double duration)
{
	for (Block *block = Blocks; block < Blocks + MAX_BLOCKS; block++)
		if (block->Exists) {
			block->Body->Integrate(duration);
			block->CalculateInternals();
		}

	if (Ball_active) {
		Ball.Body->Integrate(duration);
		Ball.CalculateInternals();
	}
}

void FractureDemo::Display() {
	const static GLfloat lightPosition[] = {0.7f,1,0.4f,0};

	RigidBodyApplication::Display();

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
	glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
	glEnable(GL_COLOR_MATERIAL);

	glEnable(GL_NORMALIZE);
	for (Block *block = Blocks; block < Blocks + MAX_BLOCKS; block++)
	    if (block->Exists) 
			block->Render();

	glDisable(GL_NORMALIZE);

	if (Ball_active) {
		glColor3f(0.4f, 0.7f, 0.4f);
		glPushMatrix();
		cyclone::Vector3 pos = Ball.Body->Pivot.Position;
		glTranslatef(pos.x, pos.y, pos.z);
		glutSolidSphere(0.25f, 16, 8);
		glPopMatrix();
	}

	glDisable(GL_LIGHTING);
	glDisable(GL_COLOR_MATERIAL);

	// Draw some scale circles
	glColor3f(0.75, 0.75, 0.75);
	for (uint32_t i = 1; i < 20; ++i) {
		glBegin(GL_LINE_LOOP);
		for (uint32_t j = 0; j < 32; ++j) {
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

	RigidBodyApplication::DrawDebug();
}

Application* getApplication() { return new FractureDemo(); }	// Called by the common demo framework to create an application object (with new) and return a pointer.