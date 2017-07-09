// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include "cyclone.h"
#include "ogl_headers.h"
#include "app.h"
#include "timing.h"

#include <stdio.h>
#include <cassert>

#define BLOB_COUNT 5
#define PLATFORM_COUNT 10
#define BLOB_RADIUS 0.4f


// Platforms are two dimensional: lines on which the particles can rest. Platforms are also contact generators for the physics.
class Platform : public cyclone::ParticleContactGenerator
{
public:
    cyclone::Vector3		start				= {};
    cyclone::Vector3		end					= {};
    cyclone::Particle		* particles			= 0;	// Holds a pointer to the particles we're checking for collisions with.

    virtual unsigned		AddContact			(cyclone::ParticleContact *contact, uint32_t limit) const;
};

unsigned Platform::AddContact(cyclone::ParticleContact *contact,
                              unsigned limit) const
{
    const static cyclone::real restitution = 0.0f;

    unsigned used = 0;
    for (unsigned i = 0; i < BLOB_COUNT; i++)
    {
        if (used >= limit) break;

        // Check for penetration
        cyclone::Vector3 toParticle = particles[i].Position - start;
        cyclone::Vector3 lineDirection = end - start;
        cyclone::real projected = toParticle * lineDirection;
        cyclone::real platformSqLength = lineDirection.squareMagnitude();
        if (projected <= 0)
        {
            // The blob is nearest to the start point
            if (toParticle.squareMagnitude() < BLOB_RADIUS*BLOB_RADIUS)
            {
                // We have a collision
                contact->contactNormal = toParticle.unit();
                contact->contactNormal.z = 0;
                contact->restitution = restitution;
                contact->particle[0] = particles + i;
                contact->particle[1] = 0;
                contact->penetration = BLOB_RADIUS - toParticle.magnitude();
                used ++;
                contact ++;
            }

        }
        else if (projected >= platformSqLength)
        {
            // The blob is nearest to the end point
            toParticle = particles[i].Position - end;
            if (toParticle.squareMagnitude() < BLOB_RADIUS*BLOB_RADIUS)
            {
                // We have a collision
                contact->contactNormal = toParticle.unit();
                contact->contactNormal.z = 0;
                contact->restitution = restitution;
                contact->particle[0] = particles + i;
                contact->particle[1] = 0;
                contact->penetration = BLOB_RADIUS - toParticle.magnitude();
                used ++;
                contact ++;
            }
        }
        else
        {
            // the blob is nearest to the middle.
            cyclone::real distanceToPlatform =
                toParticle.squareMagnitude() -
                projected*projected / platformSqLength;
            if (distanceToPlatform < BLOB_RADIUS*BLOB_RADIUS)
            {
                // We have a collision
                cyclone::Vector3 closestPoint =
                    start + lineDirection*(projected/platformSqLength);

                contact->contactNormal = (particles[i].Position-closestPoint).unit();
                contact->contactNormal.z = 0;
                contact->restitution = restitution;
                contact->particle[0] = particles + i;
                contact->particle[1] = 0;
                contact->penetration = BLOB_RADIUS - real_sqrt(distanceToPlatform);
                used ++;
                contact ++;
            }
        }
    }
    return used;
}

// A force generator for proximal attraction.
class BlobForceGenerator : public cyclone::ParticleForceGenerator {
public:
	cyclone::Particle					* particles				= 0; // Holds a pointer to the particles we might be attracting.
	cyclone::real						maxReplusion			= 0; // The maximum force used to push the particles apart.
	cyclone::real						maxAttraction			= 0; // The maximum force used to pull particles together.
	cyclone::real						minNaturalDistance		= 0; // The separation between particles where there is no force.
	cyclone::real						maxNaturalDistance		= 0; // The separation between particles where there is no force.
	cyclone::real						floatHead				= 0; // The force with which to float the head particle, if it is joined to others.
	uint32_t							maxFloat				= 0; // The maximum number of particles in the blob before the head is floated at maximum force.
	cyclone::real						maxDistance				= 0; // The separation between particles after which they 'break' apart and there is no force.

	virtual void						UpdateForce				(cyclone::Particle *particle, cyclone::real duration);
};

void BlobForceGenerator::UpdateForce(cyclone::Particle *particle,
                                      cyclone::real duration)
{
    unsigned joinCount = 0;
    for (unsigned i = 0; i < BLOB_COUNT; i++)
    {
        // Don't attract yourself
        if (particles + i == particle) continue;

        // Work out the separation distance
        cyclone::Vector3 separation = particles[i].Position - particle->Position;
        separation.z = 0.0f;
        cyclone::real distance = separation.magnitude();

        if (distance < minNaturalDistance)
        {
            // Use a repulsion force.
            distance = 1.0f - distance / minNaturalDistance;
            particle->AccumulatedForce	+= separation.unit() * (1.0f - distance) * maxReplusion * -1.0f;
            joinCount++;
        }
        else if (distance > maxNaturalDistance && distance < maxDistance) {
            // Use an attraction force.
            distance 
				= (distance		- maxNaturalDistance) 
				/ (maxDistance	- maxNaturalDistance)
				;
            particle->AccumulatedForce	+= separation.unit() * distance * maxAttraction;
            joinCount++;
        }
    }

    // If the particle is the head, and we've got a join count, then float it.
    if (particle == particles && joinCount > 0 && maxFloat > 0) {
        cyclone::real force = cyclone::real(joinCount / maxFloat) * floatHead;
        if (force > floatHead) 
			force = floatHead;
		particle->AccumulatedForce	+= {0, force, 0};
    }

}

// The main demo class definition.
class BlobDemo : public Application {
	cyclone::Particle				* blobs				= 0;
	Platform						* platforms			= 0;
	cyclone::ParticleWorld			world				;
	BlobForceGenerator				blobForceGenerator	;
	float							xAxis				;						// The control for the x-axis.
	float							yAxis				;						// The control for the y-axis.
	
	void							Reset				();
public:
	virtual							~BlobDemo			();
									BlobDemo			();						// Creates a new demo object.

	virtual const char*				GetTitle			();						// Returns the window title for the demo. 
	virtual void					Display				();						// Display the particles. 
	virtual void					Update				();						// Update the particle positions. 
	virtual void					Key					(unsigned char key);	// Handle a key press. 

	virtual void					Mouse				(int button, int state, int x, int y)	{};	// Called when GLUT detects a mouse button press.
	virtual void					MouseDrag			(int x, int y)							{};	// Called when GLUT detects a mouse drag.
};

// Method definitions
BlobDemo::BlobDemo()
	: xAxis(0)
	, yAxis(0)
	, world(PLATFORM_COUNT+BLOB_COUNT, PLATFORM_COUNT)
{
    // Create the blob storage
    blobs = new cyclone::Particle[BLOB_COUNT];
    cyclone::Random r;

    // Create the force generator
    blobForceGenerator.particles = blobs;
    blobForceGenerator.maxAttraction = 20.0f;
    blobForceGenerator.maxReplusion = 10.0f;
    blobForceGenerator.minNaturalDistance = BLOB_RADIUS*0.75f;
    blobForceGenerator.maxNaturalDistance = BLOB_RADIUS*1.5f;
    blobForceGenerator.maxDistance = BLOB_RADIUS * 2.5f;
    blobForceGenerator.maxFloat = 2;
    blobForceGenerator.floatHead = 8.0f;

    // Create the platforms
    platforms = new Platform[PLATFORM_COUNT];
    for (unsigned i = 0; i < PLATFORM_COUNT; i++)
    {
        platforms[i].start = 
            { cyclone::real(i%2)*10.0f - 5.0f
            , cyclone::real(i)*4.0f + ((i%2)?0.0f:2.0f)
			, 0
			};
        platforms[i].start.x += r.randomBinomial(2.0f);
        platforms[i].start.y += r.randomBinomial(2.0f);

        platforms[i].end = 
			{ cyclone::real(i%2)*10.0f + 5.0f
            , cyclone::real(i)*4.0f + ((i%2)?2.0f:0.0f)
			, 0
			};
        platforms[i].end.x += r.randomBinomial(2.0f);
        platforms[i].end.y += r.randomBinomial(2.0f);

        // Make sure the platform knows which particles it
        // should collide with.
        platforms[i].particles = blobs;
        world.getContactGenerators().push_back(platforms + i);
    }

    // Create the blobs.
    Platform *p = platforms + (PLATFORM_COUNT-2);
    cyclone::real fraction = (cyclone::real)1.0 / BLOB_COUNT;
    cyclone::Vector3 delta = p->end - p->start;
    for (unsigned i = 0; i < BLOB_COUNT; i++)
    {
        unsigned me = (i+BLOB_COUNT/2) % BLOB_COUNT;
		blobs[i].Position = (p->start + delta * (cyclone::real(me)*0.8f*fraction+0.1f) + cyclone::Vector3{0, 1.0f+r.randomReal(), 0});

		blobs[i].Velocity			= {};
        blobs[i].Damping			= 0.2f;
        blobs[i].Acceleration		= cyclone::Vector3::GRAVITY * 0.4f;
        blobs[i].SetMass			(1.0f);
        blobs[i].ClearAccumulator();

        world.getParticles().push_back(blobs + i);
        world.getForceRegistry().add(blobs + i, &blobForceGenerator);
    }
}

void BlobDemo::Reset()
{
    cyclone::Random r;
    Platform *p = platforms + (PLATFORM_COUNT-2);
    cyclone::real fraction = (cyclone::real)1.0 / BLOB_COUNT;
    cyclone::Vector3 delta = p->end - p->start;
    for (unsigned i = 0; i < BLOB_COUNT; i++)
    {
        unsigned me = (i+BLOB_COUNT/2) % BLOB_COUNT;
		blobs[i].Position			= (p->start + delta * (cyclone::real(me)*0.8f*fraction+0.1f) + cyclone::Vector3{0, 1.0f+r.randomReal(), 0});
		blobs[i].Velocity			= {};
        blobs[i].ClearAccumulator();
    }
}

BlobDemo::~BlobDemo()
{
    if(blobs)	
		delete blobs;
}

void BlobDemo::Display()
{
    cyclone::Vector3 pos = blobs[0].Position;

    // Clear the view port and set the camera direction
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    gluLookAt(pos.x, pos.y, 6.0,  pos.x, pos.y, 0.0,  0.0, 1.0, 0.0);

    glColor3f(0,0,0);


    glBegin(GL_LINES);
    glColor3f(0,0,1);
    for (unsigned i = 0; i < PLATFORM_COUNT; i++)
    {
        const cyclone::Vector3 &p0 = platforms[i].start;
        const cyclone::Vector3 &p1 = platforms[i].end;
        glVertex3f(p0.x, p0.y, p0.z);
        glVertex3f(p1.x, p1.y, p1.z);
    }
    glEnd();

    glColor3f(1,0,0);
    for (unsigned i = 0; i < BLOB_COUNT; i++)
    {
        const cyclone::Vector3 &p = blobs[i].Position;
        glPushMatrix();
        glTranslatef(p.x, p.y, p.z);
        glutSolidSphere(BLOB_RADIUS, 12, 12);
        glPopMatrix();
    }

    cyclone::Vector3 p = blobs[0].Position;
    cyclone::Vector3 v = blobs[0].Velocity * 0.05f;
    v.trim(BLOB_RADIUS*0.5f);
    p = p + v;
    glPushMatrix();
    glTranslatef(p.x-BLOB_RADIUS*0.2f, p.y, BLOB_RADIUS);
    glColor3f(1,1,1);
    glutSolidSphere(BLOB_RADIUS*0.2f, 8, 8);
    glTranslatef(0,0,BLOB_RADIUS*0.2f);
    glColor3f(0,0,0);
    glutSolidSphere(BLOB_RADIUS*0.1f, 8, 8);
    glTranslatef(BLOB_RADIUS*0.4f, 0, -BLOB_RADIUS*0.2f);
    glColor3f(1,1,1);
    glutSolidSphere(BLOB_RADIUS*0.2f, 8, 8);
    glTranslatef(0,0,BLOB_RADIUS*0.2f);
    glColor3f(0,0,0);
    glutSolidSphere(BLOB_RADIUS*0.1f, 8, 8);
    glPopMatrix();
}

void BlobDemo::Update()
{
	world.startFrame();	// Clear accumulators

	// Find the duration of the last frame in seconds
	float duration = (float)TimingData::get().lastFrameDuration * 0.001f;
	if (duration <= 0.0f) 
		return;
	
	// Recenter the axes
	xAxis *= pow(0.1f, duration);
	yAxis *= pow(0.1f, duration);
	
	blobs[0].AddForce(cyclone::Vector3{xAxis, yAxis, 0} * 10.0f);	// Move the controlled blob
	world.runPhysics(duration);	// Run the simulation
	
	// Bring all the particles back to 2d
	cyclone::Vector3 position;
	for (unsigned i = 0; i < BLOB_COUNT; i++)
	{
		position = blobs[i].Position;
		position.z = 0.0f;
		blobs[i].Position = position;
	}
	
	Application::Update();
}

const char* BlobDemo::GetTitle()
{
    return "Cyclone > Blob Demo";
}

void BlobDemo::Key(unsigned char key) {
    switch(key) {
    case 'w': case 'W': yAxis =  1.0;	break;
    case 's': case 'S': yAxis = -1.0;	break;
    case 'a': case 'A': xAxis = -1.0;	break;
    case 'd': case 'D': xAxis =  1.0;	break;
    case 'r': case 'R': Reset();		break;
    }
}

// Called by the common demo framework to create an application object (with new) and return a pointer.
Application* getApplication()
{
    return new BlobDemo();
}