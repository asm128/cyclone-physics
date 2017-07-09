// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include "cyclone.h"
#include "ogl_headers.h"
#include "app.h"
#include "timing.h"

#include <stdio.h>
#include <cassert>

#define ROD_COUNT 15

#define BASE_MASS 1
#define EXTRA_MASS 10

// The main demo class definition.
class PlatformDemo : public MassAggregateApplication {
	cyclone::ParticleRod				* Rods					= 0;
	
	cyclone::Vector3					MassPos					= {0,0,0.5f};
	cyclone::Vector3					MassDisplayPos			= {};
	
	void								UpdateAdditionalMass	();	// Updates particle masses to take into account the mass that's on the platform.
public:
	virtual								~PlatformDemo			()						{ if (Rods) delete[] Rods; }
										PlatformDemo			();
	
	virtual void						Display					();						// Display the particles.
	virtual void						Update					();						// Update the particle positions.
	virtual void						Key						(unsigned char key);	// Handle a key press.

	virtual const char*					GetTitle				()						{ return "Cyclone > Platform Demo"; }
	virtual void						Mouse					(int, int, int, int)	{}	// Called when GLUT detects a mouse button press.
	virtual void						MouseDrag				(int, int)				{}	// Called when GLUT detects a mouse drag.
};	

// Method definitions
PlatformDemo::PlatformDemo() : MassAggregateApplication(6)
{
    // Create the masses and connections.
    ParticleArray[0].Position			= {0,0,1	};
    ParticleArray[1].Position			= {0,0,-1	};
    ParticleArray[2].Position			= {-3,2,1	};
    ParticleArray[3].Position			= {-3,2,-1};
    ParticleArray[4].Position			= {4,2,1	};
    ParticleArray[5].Position			= {4,2,-1	};
    for (uint32_t i = 0; i < 6; i++)
    {
        ParticleArray[i].SetMass(BASE_MASS);
		ParticleArray[i].Velocity			= {};
        ParticleArray[i].Damping			= 0.9f;
        ParticleArray[i].Acceleration		= cyclone::Vector3::GRAVITY;
		ParticleArray[i].AccumulatedForce	= {};
    }

    Rods = new cyclone::ParticleRod[ROD_COUNT];

    Rods[0] .Particle[0] = &ParticleArray[0];	Rods[0] .Particle[1] = &ParticleArray[1];	Rods[0] .Length = 2;
    Rods[1] .Particle[0] = &ParticleArray[2];	Rods[1] .Particle[1] = &ParticleArray[3];	Rods[1] .Length = 2;
    Rods[2] .Particle[0] = &ParticleArray[4];	Rods[2] .Particle[1] = &ParticleArray[5];	Rods[2] .Length = 2;
			 											 										    
    Rods[3] .Particle[0] = &ParticleArray[2];	Rods[3] .Particle[1] = &ParticleArray[4];	Rods[3] .Length = 7;
    Rods[4] .Particle[0] = &ParticleArray[3];	Rods[4] .Particle[1] = &ParticleArray[5];	Rods[4] .Length = 7;
			 											 										    
    Rods[5] .Particle[0] = &ParticleArray[0];	Rods[5] .Particle[1] = &ParticleArray[2];	Rods[5] .Length = 3.606;
    Rods[6] .Particle[0] = &ParticleArray[1];	Rods[6] .Particle[1] = &ParticleArray[3];	Rods[6] .Length = 3.606;
			 											 										    
    Rods[7] .Particle[0] = &ParticleArray[0];	Rods[7] .Particle[1] = &ParticleArray[4];	Rods[7] .Length = 4.472;
    Rods[8] .Particle[0] = &ParticleArray[1];	Rods[8] .Particle[1] = &ParticleArray[5];	Rods[8] .Length = 4.472;
			 											 										    
    Rods[9] .Particle[0] = &ParticleArray[0];	Rods[9] .Particle[1] = &ParticleArray[3];	Rods[9] .Length = 4.123;
    Rods[10].Particle[0] = &ParticleArray[2];	Rods[10].Particle[1] = &ParticleArray[5];	Rods[10].Length = 7.28;
    Rods[11].Particle[0] = &ParticleArray[4];	Rods[11].Particle[1] = &ParticleArray[1];	Rods[11].Length = 4.899;
    Rods[12].Particle[0] = &ParticleArray[1];	Rods[12].Particle[1] = &ParticleArray[2];	Rods[12].Length = 4.123;
    Rods[13].Particle[0] = &ParticleArray[3];	Rods[13].Particle[1] = &ParticleArray[4];	Rods[13].Length = 7.28;
    Rods[14].Particle[0] = &ParticleArray[5];	Rods[14].Particle[1] = &ParticleArray[0];	Rods[14].Length = 4.899;

    for (uint32_t i = 0; i < ROD_COUNT; i++)
        World.ContactGenerators.push_back(&Rods[i]);

    UpdateAdditionalMass();
}

void PlatformDemo::UpdateAdditionalMass() {
	for (uint32_t i = 2; i < 6; i++)
		ParticleArray[i].SetMass(BASE_MASS);

	// Find the coordinates of the mass as an index and proportion
	double xp = MassPos.x;
	if (xp < 0) xp = 0;
	if (xp > 1) xp = 1;

	double zp = MassPos.z;
	if (zp < 0) zp = 0;
	if (zp > 1) zp = 1;

	// Calculate where to draw the mass
	MassDisplayPos.clear();	

	// Add the proportion to the correct masses
	ParticleArray[2].SetMass(BASE_MASS + EXTRA_MASS*(1-xp)*(1-zp));
	MassDisplayPos.addScaledVector(ParticleArray[2].Position, (1-xp)*(1-zp));

	if (xp > 0) {
	    ParticleArray[4].SetMass(BASE_MASS + EXTRA_MASS*xp*(1-zp));
	    MassDisplayPos.addScaledVector(ParticleArray[4].Position, xp*(1-zp));

	    if (zp > 0) {
	        ParticleArray[5].SetMass(BASE_MASS + EXTRA_MASS*xp*zp);
	        MassDisplayPos.addScaledVector(ParticleArray[5].Position, xp*zp);
	    }
	}
	if (zp > 0) {
	    ParticleArray[3].SetMass(BASE_MASS + EXTRA_MASS*(1-xp)*zp);
	    MassDisplayPos.addScaledVector(ParticleArray[3].Position, (1-xp)*zp);
	}
}

void PlatformDemo::Display()
{
    MassAggregateApplication::Display();

    glBegin(GL_LINES);
    glColor3f(0,0,1);
    for (uint32_t i = 0; i < ROD_COUNT; i++)
    {
        cyclone::Particle **particles = Rods[i].Particle;
        const cyclone::Vector3 &p0 = particles[0]->Position;
        const cyclone::Vector3 &p1 = particles[1]->Position;
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

void PlatformDemo::Update() {
    MassAggregateApplication::Update();
    UpdateAdditionalMass();
}

void PlatformDemo::Key(unsigned char key) {
    switch(key) {
    case 'w': case 'W':	MassPos.z += 0.1f;	if (MassPos.z > 1.0f) MassPos.z = 1.0f;	break;
    case 's': case 'S':	MassPos.z -= 0.1f;	if (MassPos.z < 0.0f) MassPos.z = 0.0f;	break;
    case 'a': case 'A':	MassPos.x -= 0.1f;	if (MassPos.x < 0.0f) MassPos.x = 0.0f;	break;
    case 'd': case 'D':	MassPos.x += 0.1f;	if (MassPos.x > 1.0f) MassPos.x = 1.0f;	break;
    default:
        MassAggregateApplication::Key(key);
    }
}

// Called by the common demo framework to create an application object (with new) and return a pointer.
Application* getApplication() { return new PlatformDemo(); }