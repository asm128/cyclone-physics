// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include "cyclone.h"
#include "ogl_headers.h"
#include "app.h"
#include "timing.h"

#include <stdio.h>
#include <cassert>

#define ROD_COUNT 6
#define CABLE_COUNT 10
#define SUPPORT_COUNT 12

#define BASE_MASS 1
#define EXTRA_MASS 10

// The main demo class definition.
class BridgeDemo : public MassAggregateApplication {
	::cyclone::ParticleCableConstraint	* supports					= 0;
	::cyclone::ParticleCable			* cables					= 0;
	::cyclone::ParticleRod				* rods						= 0;
	
	::cyclone::Vector3					MassPos						= {0, 0, 0.5f};
	::cyclone::Vector3					MassDisplayPos				= {};

	void								UpdateAdditionalMass		();	// Updates particle masses to take into account the mass that's crossing the bridge.

public:
	virtual								~BridgeDemo					();
										BridgeDemo					();

	virtual const char*					GetTitle					()									{ return "Cyclone > Bridge Demo"; }
	virtual void						Display						();	// Display the particles.
	virtual void						Update						();	// Update the particle positions.
	virtual void						Key							(unsigned char key);	// Handle a key press.
	virtual void						Mouse						(int, int, int, int)				{}	// Called when GLUT detects a mouse button press.
	virtual void						MouseDrag					(int, int)							{}	// Called when GLUT detects a mouse drag.
};

// Method definitions
BridgeDemo::BridgeDemo() : MassAggregateApplication(12) {
    // Create the masses and connections.
    for (unsigned i = 0; i < 12; i++)
    {
        //unsigned x = (i%12)/2;
        ParticleArray[i].Position = {
            double(i/2)*2.0f-5.0f,
            4,
            double(i%2)*2.0f-1.0f
		};
		ParticleArray[i].Velocity			= {};
        ParticleArray[i].Damping			= 0.9f;
        ParticleArray[i].Acceleration		= cyclone::Vector3::GRAVITY;
		ParticleArray[i].AccumulatedForce	= {};
    }

    // Add the links
    cables = new cyclone::ParticleCable[CABLE_COUNT];
    for (unsigned i = 0; i < 10; i++) {
        cables[i].Particle[0]				= &ParticleArray[i];
        cables[i].Particle[1]				= &ParticleArray[i+2];
        cables[i].MaxLength					= 1.9f;
        cables[i].Restitution				= 0.3f;
        World.ContactGenerators.push_back(&cables[i]);
    }

    supports = new cyclone::ParticleCableConstraint[SUPPORT_COUNT];
    for (unsigned i = 0; i < SUPPORT_COUNT; i++)
    {
        supports[i].Particle = ParticleArray + i;
        supports[i].Anchor = 
            {	double(i/2)*2.2f-5.5f
            ,	6
            ,	double(i%2)*1.6f-0.8f
            };
        
		if (i < 6)	supports[i].MaxLength = double(i/2)*0.5f + 3.0f;
        else		supports[i].MaxLength = 5.5f - double(i/2)*0.5f;
        
		supports[i].Restitution = 0.5f;
        World.ContactGenerators.push_back(&supports[i]);
    }

    rods = new cyclone::ParticleRod[ROD_COUNT];
    for (unsigned i = 0; i < 6; i++)
    {
        rods[i].Particle[0] = &ParticleArray[i*2];
        rods[i].Particle[1] = &ParticleArray[i*2+1];
        rods[i].Length = 2;
        World.ContactGenerators.push_back(&rods[i]);
    }

    UpdateAdditionalMass();
}

BridgeDemo::~BridgeDemo() {
    if (cables)		delete[] cables;
    if (rods)		delete[] rods;
    if (supports)	delete[] supports;
}

void BridgeDemo::UpdateAdditionalMass()
{
    for (unsigned i = 0; i < 12; i++)
        ParticleArray[i].SetMass(BASE_MASS);

    // Find the coordinates of the mass as an index and proportion
    int x = int(MassPos.x);
    double xp = real_fmod(MassPos.x, double(1.0f));
    if (x < 0)
    {
        x = 0;
        xp = 0;
    }
    if (x >= 5)
    {
        x = 5;
        xp = 0;
    }

    int z = int(MassPos.z);
    double zp = real_fmod(MassPos.z, double(1.0f));
    if (z < 0)
    {
        z = 0;
        zp = 0;
    }
    if (z >= 1)
    {
        z = 1;
        zp = 0;
    }

    // Calculate where to draw the mass
    MassDisplayPos.clear();

    // Add the proportion to the correct masses
    ParticleArray[x*2+z].SetMass(BASE_MASS + EXTRA_MASS*(1-xp)*(1-zp));
    MassDisplayPos.addScaledVector(ParticleArray[x*2+z].Position, (1-xp)*(1-zp));

    if (xp > 0) {
        ParticleArray[x*2+z+2].SetMass(BASE_MASS + EXTRA_MASS*xp*(1-zp));
        MassDisplayPos.addScaledVector(ParticleArray[x*2+z+2].Position, xp*(1-zp));

        if (zp > 0) {
            ParticleArray[x*2+z+3].SetMass(BASE_MASS + EXTRA_MASS*xp*zp);
            MassDisplayPos.addScaledVector(ParticleArray[x*2+z+3].Position, xp*zp);
        }
    }
    if (zp > 0) {
        ParticleArray[x*2+z+1].SetMass(BASE_MASS + EXTRA_MASS*(1-xp)*zp);
        MassDisplayPos.addScaledVector(ParticleArray[x*2+z+1].Position, (1-xp)*zp);
    }
}

void BridgeDemo::Display()
{
    MassAggregateApplication::Display();

    glBegin(GL_LINES);
    glColor3f(0,0,1);
    for (unsigned i = 0; i < ROD_COUNT; i++)
    {
        cyclone::Particle **particles = rods[i].Particle;
        const cyclone::Vector3 &p0 = particles[0]->Position;
        const cyclone::Vector3 &p1 = particles[1]->Position;
        glVertex3f(p0.x, p0.y, p0.z);
        glVertex3f(p1.x, p1.y, p1.z);
    }

    glColor3f(0,1,0);
    for (unsigned i = 0; i < CABLE_COUNT; i++)
    {
        cyclone::Particle **particles = cables[i].Particle;
        const cyclone::Vector3 &p0 = particles[0]->Position;
        const cyclone::Vector3 &p1 = particles[1]->Position;
        glVertex3f(p0.x, p0.y, p0.z);
        glVertex3f(p1.x, p1.y, p1.z);
    }

    glColor3f(0.7f, 0.7f, 0.7f);
    for (unsigned i = 0; i < SUPPORT_COUNT; i++)
    {
        const cyclone::Vector3 &p0 = supports[i].Particle->Position;
        const cyclone::Vector3 &p1 = supports[i].Anchor;
        glVertex3f(p0.x, p0.y, p0.z);
        glVertex3f(p1.x, p1.y, p1.z);
    }
    glEnd();

    glColor3f(1,0,0);
    glPushMatrix();
    glTranslatef(MassDisplayPos.x, MassDisplayPos.y+0.25f, MassDisplayPos.z);
    glutSolidSphere(0.25f, 20, 10);
    glPopMatrix();
}

void BridgeDemo::Update()
{
    MassAggregateApplication::Update();

    UpdateAdditionalMass();
}


void BridgeDemo::Key(unsigned char key)
{
    switch(key) {
    case 's': case 'S':	MassPos.z += 0.1f;	if (MassPos.z > 1.0f) MassPos.z = 1.0f;	break;
    case 'w': case 'W':	MassPos.z -= 0.1f;	if (MassPos.z < 0.0f) MassPos.z = 0.0f;	break;
    case 'a': case 'A':	MassPos.x -= 0.1f;	if (MassPos.x < 0.0f) MassPos.x = 0.0f;	break;
    case 'd': case 'D':	MassPos.x += 0.1f;	if (MassPos.x > 5.0f) MassPos.x = 5.0f;	break;
    default:
        MassAggregateApplication::Key(key);
    }
}

Application* getApplication() { return new BridgeDemo(); }		// Called by the common demo framework to create an application object (with new) and return a pointer.
