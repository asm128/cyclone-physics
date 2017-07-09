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
									Bone							()									{ Body = &_boneBody; }

    // We use a sphere to collide bone on bone to allow some limited interpenetration.
    cyclone::CollisionSphere		getCollisionSphere				()					const			{
        cyclone::CollisionSphere sphere;
        sphere.Body = Body;
        sphere.Radius = HalfSize.x;
        sphere.Offset = cyclone::Matrix4();
        if (HalfSize.y < sphere.Radius) sphere.Radius = HalfSize.y;
        if (HalfSize.z < sphere.Radius) sphere.Radius = HalfSize.z;
        sphere.CalculateInternals();
        return sphere;
    }

	// Draws the bone.
    void							Render							()									{
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

    /** Sets the bone to a specific location. */
    void setState(const cyclone::Vector3 &position,
                  const cyclone::Vector3 &extents)
    {
        Body->Position		= position;
        Body->Orientation	= {};
        Body->Velocity		= {};
        Body->Rotation		= {};
        HalfSize			= extents;

        double mass = HalfSize.x * HalfSize.y * HalfSize.z * 8.0f;
        Body->setMass(mass);

        cyclone::Matrix3 tensor;
        tensor.setBlockInertiaTensor(HalfSize, mass);
        Body->setInertiaTensor(tensor);

        Body->LinearDamping		= 0.95f;
        Body->AngularDamping	= 0.8f;
        Body->clearAccumulators();
        Body->Acceleration = ::cyclone::Vector3::GRAVITY;

        Body->setCanSleep(false);
        Body->setAwake();

        Body->calculateDerivedData();
        CalculateInternals();
    }

};

// The main demo class definition.
class RagdollDemo : public RigidBodyApplication {
	cyclone::Random		random;
	Bone				bones	[NUM_BONES];	// Holds the bone bodies.	
	cyclone::Joint		joints	[NUM_JOINTS];	// Holds the joints.		

	virtual void		GenerateContacts			();	// Processes the contact generation code. 
	virtual void		UpdateObjects				(double duration);	// Processes the objects in the simulation forward in time.
	virtual void		Reset						();	// Resets the position of all the bones. 
public:
						RagdollDemo					();	// Creates a new demo object.

	virtual const char*	GetTitle					()											{ return "Cyclone > Ragdoll Demo"; }
	virtual void		InitGraphics				();	// Sets up the rendering.
	virtual void		Display						();	// Display the particle positions.
};

// Method definitions
RagdollDemo::RagdollDemo()
	: RigidBodyApplication()
{
	// -- Set up the bone hierarchy. --
	joints[0]	.Set(bones[0]	.Body, {0, 1.07f, 0}		, bones[1]	.Body, {0, -1.07f, 0}, 0.15f);	// Right Knee
	joints[1]	.Set(bones[2]	.Body, {0, 1.07f, 0}		, bones[3]	.Body, {0, -1.07f, 0}, 0.15f);	// Left Knee
	joints[2]	.Set(bones[9]	.Body, {0, 0.96f, 0}		, bones[8]	.Body, {0, -0.96f, 0}, 0.15f);	// Right elboW
	joints[3]	.Set(bones[11]	.Body, {0, 0.96f, 0}		, bones[10]	.Body, {0, -0.96f, 0}, 0.15f);	// Left elbow
	
	joints[7]	.Set(bones[1]	.Body, {0, 1.066f, 0}		, bones[4]	.Body, {0, -0.458f, -0.5f}	, 0.15f);	// Right hip
	joints[8]	.Set(bones[3]	.Body, {0, 1.066f, 0}		, bones[4]	.Body, {0, -0.458f, 0.5f}	, 0.105f);	// Left Hip
	joints[9]	.Set(bones[6]	.Body, {0, 0.367f, -0.8f}	, bones[8]	.Body, {0, 0.888f, 0.32f}	, 0.15f);	// Right shouLder  
	joints[10]	.Set(bones[6]	.Body, {0, 0.367f, 0.8f}	, bones[10]	.Body, {0, 0.888f, -0.32f}	, 0.15f);	// Left shoulDEr
	
	// Stomach to Waist
	joints[4]	.Set(bones[4]	.Body, { 0.054f,  0.50f, 0}	, bones[5]	.Body, {-0.043f, -0.45f, 0}	, 0.15f);	
	joints[5]	.Set(bones[5]	.Body, {-0.043f, 0.411f, 0}	, bones[6]	.Body, {0, -0.411f, 0}		, 0.15f);
	joints[6]	.Set(bones[6]	.Body, {0, 0.521f, 0}		, bones[7]	.Body, {0, -0.752f, 0}		, 0.15f);

	Reset();	// Set up the initial positions
}

void RagdollDemo::GenerateContacts()
{
    // Create the ground plane data
    cyclone::CollisionPlane				plane;
	plane.Direction					= {0,1,0};
    plane.Offset					= 0;

    // Set up the collision data structure
    CData.Reset(MaxContacts);
    CData.Friction					= (double)0.9;
    CData.Restitution				= (double)0.6;
    CData.Tolerance					= (double)0.1;

    // Perform exhaustive collision detection on the ground plane
    cyclone::Matrix4					transform	, otherTransform;
    cyclone::Vector3					position	, otherPosition;
    for (Bone *bone = bones; bone < bones+NUM_BONES; bone++) {
        // Check for collisions with the ground plane
        if (!CData.HasMoreContacts()) 
			return;

        cyclone::CollisionDetector::boxAndHalfSpace(*bone, plane, &CData);
        cyclone::CollisionSphere		boneSphere	= bone->getCollisionSphere();
        for (Bone *other = bone+1; other < bones+NUM_BONES; other++) {	// Check for collisions with each other box
            if (!CData.HasMoreContacts()) return;

            cyclone::CollisionSphere otherSphere = other->getCollisionSphere();
            cyclone::CollisionDetector::sphereAndSphere(
                boneSphere,
                otherSphere,
                &CData
                );
        }
    }

    // Check for joint violation
    for (cyclone::Joint *joint = joints; joint < joints+NUM_JOINTS; joint++)
    {
        if (!CData.HasMoreContacts()) return;
        unsigned added = joint->AddContact(CData.Contacts, CData.ContactsLeft);
        CData.AddContacts(added);
    }
}

void RagdollDemo::Reset()
{
	bones[0]	.setState({ 0.000, 0.993, -0.500}, {0.301, 1.000, 0.234});
	bones[1]	.setState({ 0.000, 3.159, -0.560}, {0.301, 1.000, 0.234});
	bones[2]	.setState({ 0.000, 0.993,  0.500}, {0.301, 1.000, 0.234});
	bones[3]	.setState({ 0.000, 3.150,  0.560}, {0.301, 1.000, 0.234});

	bones[4]	.setState({-0.054, 4.683,  0.013}, {0.415, 0.392, 0.690});
	bones[5]	.setState({ 0.043, 5.603,  0.013}, {0.301, 0.367, 0.693});
	bones[6]	.setState({ 0.000, 6.485,  0.013}, {0.435, 0.367, 0.786});
	bones[7]	.setState({ 0.000, 7.759,  0.013}, {0.450, 0.598, 0.421});

	bones[8]	.setState({ 0.000, 5.946, -1.066}, {0.267, 0.888, 0.207});
	bones[9]	.setState({ 0.000, 4.024, -1.066}, {0.267, 0.888, 0.207});
	bones[10]	.setState({ 0.000, 5.946,  1.066}, {0.267, 0.888, 0.207});
	bones[11]	.setState({ 0.000, 4.024,  1.066}, {0.267, 0.888, 0.207});

	double strength = -random.randomReal(500.0f, 1000.0f);
	for (unsigned i = 0; i < NUM_BONES; i++)
		bones[i].Body->addForceAtBodyPoint( {strength, 0, 0}, {});

	bones[6].Body->addForceAtBodyPoint(
	    { strength, 0, random.randomBinomial(1000.0f)},
		{ random.randomBinomial(4.0f), random.randomBinomial(3.0f), 0}
	    );

	CData.ContactCount = 0;	// Reset the contacts
}

void RagdollDemo::UpdateObjects(double duration)
{
    for (Bone *bone = bones; bone < bones+NUM_BONES; bone++)
    {
        bone->Body->integrate(duration);
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
    for (unsigned i = 0; i < NUM_BONES; i++)
    {
        bones[i].Render();
    }
    glDisable(GL_NORMALIZE);

    glDisable(GL_LIGHTING);
    glDisable(GL_COLOR_MATERIAL);

    glDisable(GL_DEPTH_TEST);
    glBegin(GL_LINES);
    for (unsigned i = 0; i < NUM_JOINTS; i++)
    {
        cyclone::Joint *joint = joints + i;
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

    RigidBodyApplication::DrawDebug();
}

// Called by the common demo framework to create an application object (with new) and return a pointer.
Application* getApplication() {
    return new RagdollDemo();
}