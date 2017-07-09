// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include "cyclone.h"
#include "ogl_headers.h"
#include "app.h"
#include "timing.h"

#include <stdio.h>

#define NUM_BONES 12
#define NUM_JOINTS 11

class Bone : public cyclone::CollisionBox
{
	::cyclone::RigidBody			_boneBody;
public:
									Bone								()																					{ Body = &_boneBody; }

    // We use a sphere to collide bone on bone to allow some limited interpenetration.
    cyclone::CollisionSphere		getCollisionSphere					()																	const			{
        cyclone::CollisionSphere			sphere;
        sphere.Body						= Body;
        sphere.Radius					= HalfSize.x;
        sphere.Offset					= cyclone::Matrix4();
        if (HalfSize.y < sphere.Radius) sphere.Radius = HalfSize.y;
        if (HalfSize.z < sphere.Radius) sphere.Radius = HalfSize.z;
        sphere.CalculateInternals();
        return sphere;
    }

	// Draws the bone.
    void							Render								()																					{
        // Get the OpenGL transformation
		GLfloat								mat	[16];
        Body->getGLTransform(mat);

        if (Body->IsAwake) 
			glColor3f(0.5f, 0.3f, 0.3f);
        else 
			glColor3f(0.3f, 0.3f, 0.5f);

        glPushMatrix();
        glMultMatrixf(mat);
        glScalef(HalfSize.x*2, HalfSize.y*2, HalfSize.z*2);
        glutSolidCube(1.0f);
        glPopMatrix();
    }

    // Sets the bone to a specific location.
    void							setState							(const cyclone::Vector3 &position, const cyclone::Vector3 &extents)					{
		Body->Pivot.Position			= position;
		Body->Pivot.Orientation			= {};
		Body->Force.Velocity			= {};
		Body->Force.Rotation			= {};
		HalfSize						= extents;

		double								mass								= HalfSize.x * HalfSize.y * HalfSize.z * 8.0f;
		Body->Mass.setMass(mass);

		cyclone::Matrix3					tensor;
		tensor.setBlockInertiaTensor(HalfSize, mass);
		Body->Mass.setInertiaTensor(tensor);

		Body->Mass.LinearDamping		= 0.95f;
		Body->Mass.AngularDamping		= 0.8f;
		Body->clearAccumulators();
		Body->Force.Acceleration		= ::cyclone::Vector3::GRAVITY;

		Body->setCanSleep(false);
		Body->setAwake();

		Body->CalculateDerivedData();
		CalculateInternals();
    }

};

// The main demo class definition.
class RagdollDemo : public RigidBodyApplication {
	cyclone::Random					Random								= {};
	Bone							Bones	[NUM_BONES]					= {};	// Holds the bone bodies.	
	cyclone::Joint					Joints	[NUM_JOINTS]				= {};	// Holds the joints.		

	virtual void					GenerateContacts					();	// Processes the contact generation code. 
	virtual void					UpdateObjects						(double duration);	// Processes the objects in the simulation forward in time.
	virtual void					Reset								();	// Resets the position of all the bones. 
public:
									RagdollDemo							();	// Creates a new demo object.

	virtual void					InitGraphics						();	// Sets up the rendering.
	virtual void					Display								();	// Display the particle positions.

	virtual const char*				GetTitle							()																					{ return "Cyclone > Ragdoll Demo"; }
};

// Method definitions
									RagdollDemo::RagdollDemo			()																					{
	// -- Set up the bone hierarchy. --
	Joints[0]	.Set(Bones[0]	.Body, {0, 1.07f, 0}		, Bones[1]	.Body, {0, -1.07f, 0}, 0.15f);	// Right Knee
	Joints[1]	.Set(Bones[2]	.Body, {0, 1.07f, 0}		, Bones[3]	.Body, {0, -1.07f, 0}, 0.15f);	// Left Knee
	Joints[2]	.Set(Bones[9]	.Body, {0, 0.96f, 0}		, Bones[8]	.Body, {0, -0.96f, 0}, 0.15f);	// Right elboW
	Joints[3]	.Set(Bones[11]	.Body, {0, 0.96f, 0}		, Bones[10]	.Body, {0, -0.96f, 0}, 0.15f);	// Left elbow
	
	Joints[7]	.Set(Bones[1]	.Body, {0, 1.066f, 0}		, Bones[4]	.Body, {0, -0.458f, -0.5f}	, 0.15f);	// Right hip
	Joints[8]	.Set(Bones[3]	.Body, {0, 1.066f, 0}		, Bones[4]	.Body, {0, -0.458f, 0.5f}	, 0.105f);	// Left Hip
	Joints[9]	.Set(Bones[6]	.Body, {0, 0.367f, -0.8f}	, Bones[8]	.Body, {0, 0.888f, 0.32f}	, 0.15f);	// Right shouLder  
	Joints[10]	.Set(Bones[6]	.Body, {0, 0.367f, 0.8f}	, Bones[10]	.Body, {0, 0.888f, -0.32f}	, 0.15f);	// Left shoulDEr
	
	// Stomach to WaiSt
	Joints[4]	.Set(Bones[4]	.Body, { 0.054f,  0.50f, 0}	, Bones[5]	.Body, {-0.043f, -0.45f, 0}	, 0.15f);	
	Joints[5]	.Set(Bones[5]	.Body, {-0.043f, 0.411f, 0}	, Bones[6]	.Body, {0, -0.411f, 0}		, 0.15f);
	Joints[6]	.Set(Bones[6]	.Body, {0, 0.521f, 0}		, Bones[7]	.Body, {0, -0.752f, 0}		, 0.15f);

	Reset();	// Set up the initial positions
}

void								RagdollDemo::GenerateContacts		()																					{
	// Create the ground plane data
	cyclone::CollisionPlane					plane;
	plane.Direction						= {0,1,0};
	plane.Offset						= 0;

	// Set up the collision data structure
	Collisions.Reset(MaxContacts);
	Collisions.Friction					= (double)0.9;
	Collisions.Restitution				= (double)0.6;
	Collisions.Tolerance				= (double)0.1;

	// Perform exhaustive collision detection on the ground plane
	cyclone::Matrix4					transform	, otherTransform;
	cyclone::Vector3					position	, otherPosition;
	for (Bone *bone = Bones; bone < Bones + NUM_BONES; bone++) {
		// Check for collisions with the ground plane
		if (!Collisions.HasMoreContacts()) 
			return;

		cyclone::CollisionDetector::boxAndHalfSpace(*bone, plane, &Collisions);
		cyclone::CollisionSphere			boneSphere			= bone->getCollisionSphere();
		for (Bone *other = bone+1; other < Bones + NUM_BONES; other++) {	// Check for collisions with each other box
			if (!Collisions.HasMoreContacts()) 
				return;
			cyclone::CollisionSphere		otherSphere			= other->getCollisionSphere();
			cyclone::CollisionDetector::sphereAndSphere(boneSphere, otherSphere, &Collisions);
	    }
	}

	// Check for joint violation
	for (cyclone::Joint *joint = Joints; joint < Joints + NUM_JOINTS; joint++) {
		if (!Collisions.HasMoreContacts()) 
			return;
		uint32_t added = joint->AddContact(Collisions.Contacts, Collisions.ContactsLeft);
		Collisions.AddContacts(added);
	}
}

void RagdollDemo::Reset()
{
	Bones[0]	.setState({ 0.000, 0.993, -0.500}, {0.301, 1.000, 0.234});
	Bones[1]	.setState({ 0.000, 3.159, -0.560}, {0.301, 1.000, 0.234});
	Bones[2]	.setState({ 0.000, 0.993,  0.500}, {0.301, 1.000, 0.234});
	Bones[3]	.setState({ 0.000, 3.150,  0.560}, {0.301, 1.000, 0.234});

	Bones[4]	.setState({-0.054, 4.683,  0.013}, {0.415, 0.392, 0.690});
	Bones[5]	.setState({ 0.043, 5.603,  0.013}, {0.301, 0.367, 0.693});
	Bones[6]	.setState({ 0.000, 6.485,  0.013}, {0.435, 0.367, 0.786});
	Bones[7]	.setState({ 0.000, 7.759,  0.013}, {0.450, 0.598, 0.421});

	Bones[8]	.setState({ 0.000, 5.946, -1.066}, {0.267, 0.888, 0.207});
	Bones[9]	.setState({ 0.000, 4.024, -1.066}, {0.267, 0.888, 0.207});
	Bones[10]	.setState({ 0.000, 5.946,  1.066}, {0.267, 0.888, 0.207});
	Bones[11]	.setState({ 0.000, 4.024,  1.066}, {0.267, 0.888, 0.207});

	double strength = -Random.RandomReal(500.0f, 1000.0f);
	for (uint32_t i = 0; i < NUM_BONES; ++i)
		Bones[i].Body->addForceAtBodyPoint( {strength, 0, 0}, {});

	Bones[6].Body->addForceAtBodyPoint(
		{ strength, 0, Random.RandomBinomial(1000.0f)},
		{ Random.RandomBinomial(4.0f), Random.RandomBinomial(3.0f), 0}
		);

	Collisions.ContactCount = 0;	// Reset the contacts
}

void RagdollDemo::UpdateObjects(double duration) {
    for (Bone *bone = Bones; bone < Bones + NUM_BONES; ++bone) {
        bone->Body->Integrate(duration);
        bone->CalculateInternals();
    }
}

void RagdollDemo::InitGraphics()
{
    GLfloat lightAmbient[] = {0.8f,0.8f,0.8f,1.0f};
    GLfloat lightDiffuse[] = {0.9f,0.95f,1.0f,1.0f};

    glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse);

    glEnable(GL_LIGHT0);

    Application::InitGraphics();
}

void RagdollDemo::Display()
{
    const static GLfloat lightPosition[] = {0.7f,-1,0.4f,0};
    const static GLfloat lightPositionMirror[] = {0.7f,1,0.4f,0};

    RigidBodyApplication::Display();

    // Render the bones
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);

    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
    glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
    glEnable(GL_COLOR_MATERIAL);

    glEnable(GL_NORMALIZE);
    glColor3f(1,0,0);
    for (uint32_t i = 0; i < NUM_BONES; i++)
		Bones[i].Render();

	glDisable(GL_NORMALIZE);

    glDisable(GL_LIGHTING);
    glDisable(GL_COLOR_MATERIAL);

    glDisable(GL_DEPTH_TEST);
    glBegin(GL_LINES);
    for (uint32_t i = 0; i < NUM_JOINTS; i++) {
        cyclone::Joint *joint = Joints + i;
        cyclone::Vector3 a_pos = joint->Body[0]->getPointInWorldSpace(joint->Position[0]);
        cyclone::Vector3 b_pos = joint->Body[1]->getPointInWorldSpace(joint->Position[1]);
        double length = (b_pos - a_pos).magnitude();

        if (length > joint->Error) 
			glColor3f(1,0,0);
        else 
			glColor3f(0,1,0);

        glVertex3f(a_pos.x, a_pos.y, a_pos.z);
        glVertex3f(b_pos.x, b_pos.y, b_pos.z);
    }
    glEnd();
    glEnable(GL_DEPTH_TEST);

    // Draw some scale circles
    glColor3f(0.75, 0.75, 0.75);
    for (uint32_t i = 1; i < 20; i++) {
        glBegin(GL_LINE_LOOP);
        for (uint32_t j = 0; j < 32; j++) {
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

Application* getApplication() { return new RagdollDemo(); }	// Called by the common demo framework to create an application object (with new) and return a pointer.