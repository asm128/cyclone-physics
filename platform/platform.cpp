/*
 * The platform demo.
 *
 * Part of the Cyclone physics system.
 *
 * Copyright (c) Icosagon 2003. All Rights Reserved.
 *
 * This software is distributed under licence. Use of this software
 * implies agreement with all terms and conditions of the accompanying
 * software licence.
 */

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
	cyclone::ParticleRod				* Rods;
	
	cyclone::Vector3					MassPos;
	cyclone::Vector3					MassDisplayPos;
	
	void								UpdateAdditionalMass	();	// Updates particle masses to take into account the mass that's on the platform.
public:
	virtual								~PlatformDemo			();
										PlatformDemo			();
	
	virtual const char*					GetTitle				();	// Returns the window title for the demo.
	virtual void						Display					();						// Display the particles.
	virtual void						Update					();						// Update the particle positions.
	virtual void						Key						(unsigned char key);	// Handle a key press.
};

// Method definitions
PlatformDemo::PlatformDemo()
	: MassAggregateApplication(6)
	, Rods(0)
	, MassPos(0,0,0.5f)
{
    // Create the masses and connections.
    ParticleArray[0].Position = {0,0,1	};
    ParticleArray[1].Position = {0,0,-1	};
    ParticleArray[2].Position = {-3,2,1	};
    ParticleArray[3].Position = {-3,2,-1};
    ParticleArray[4].Position = {4,2,1	};
    ParticleArray[5].Position = {4,2,-1	};
    for (unsigned i = 0; i < 6; i++)
    {
        ParticleArray[i].setMass(BASE_MASS);
		ParticleArray[i].Velocity		= {};
        ParticleArray[i].Damping		= 0.9f;
        ParticleArray[i].Acceleration	= cyclone::Vector3::GRAVITY;
        ParticleArray[i].clearAccumulator();
    }

    Rods = new cyclone::ParticleRod[ROD_COUNT];

    Rods[0].particle[0] = &ParticleArray[0];	Rods[0].particle[1] = &ParticleArray[1];	Rods[0].length = 2;
    Rods[1].particle[0] = &ParticleArray[2];	Rods[1].particle[1] = &ParticleArray[3];	Rods[1].length = 2;
    Rods[2].particle[0] = &ParticleArray[4];	Rods[2].particle[1] = &ParticleArray[5];	Rods[2].length = 2;

    Rods[3].particle[0] = &ParticleArray[2];	Rods[3].particle[1] = &ParticleArray[4];	Rods[3].length = 7;
    Rods[4].particle[0] = &ParticleArray[3];	Rods[4].particle[1] = &ParticleArray[5];	Rods[4].length = 7;

    Rods[5].particle[0] = &ParticleArray[0];	Rods[5].particle[1] = &ParticleArray[2];	Rods[5].length = 3.606;
    Rods[6].particle[0] = &ParticleArray[1];	Rods[6].particle[1] = &ParticleArray[3];	Rods[6].length = 3.606;

    Rods[7].particle[0] = &ParticleArray[0];	Rods[7].particle[1] = &ParticleArray[4];	Rods[7].length = 4.472;
    Rods[8].particle[0] = &ParticleArray[1];	Rods[8].particle[1] = &ParticleArray[5];	Rods[8].length = 4.472;

    Rods[9].particle[0] = &ParticleArray[0];	Rods[9].particle[1] = &ParticleArray[3];	Rods[9].length = 4.123;
    Rods[10].particle[0] = &ParticleArray[2];	Rods[10].particle[1] = &ParticleArray[5];	Rods[10].length = 7.28;
    Rods[11].particle[0] = &ParticleArray[4];	Rods[11].particle[1] = &ParticleArray[1];	Rods[11].length = 4.899;
    Rods[12].particle[0] = &ParticleArray[1];	Rods[12].particle[1] = &ParticleArray[2];	Rods[12].length = 4.123;
    Rods[13].particle[0] = &ParticleArray[3];	Rods[13].particle[1] = &ParticleArray[4];	Rods[13].length = 7.28;
    Rods[14].particle[0] = &ParticleArray[5];	Rods[14].particle[1] = &ParticleArray[0];	Rods[14].length = 4.899;

    for (unsigned i = 0; i < ROD_COUNT; i++)
        World.getContactGenerators().push_back(&Rods[i]);

    UpdateAdditionalMass();
}

PlatformDemo::~PlatformDemo() {
	if (Rods) 
		delete[] Rods;
}

void PlatformDemo::UpdateAdditionalMass() {
	for (unsigned i = 2; i < 6; i++)
		ParticleArray[i].setMass(BASE_MASS);

    // Find the coordinates of the mass as an index and proportion
    cyclone::real xp = MassPos.x;
    if (xp < 0) xp = 0;
    if (xp > 1) xp = 1;

    cyclone::real zp = MassPos.z;
    if (zp < 0) zp = 0;
    if (zp > 1) zp = 1;

    // Calculate where to draw the mass
    MassDisplayPos.clear();

    // Add the proportion to the correct masses
    ParticleArray[2].setMass(BASE_MASS + EXTRA_MASS*(1-xp)*(1-zp));
    MassDisplayPos.addScaledVector(ParticleArray[2].Position, (1-xp)*(1-zp));

    if (xp > 0)
    {
        ParticleArray[4].setMass(BASE_MASS + EXTRA_MASS*xp*(1-zp));
        MassDisplayPos.addScaledVector(ParticleArray[4].Position, xp*(1-zp));

        if (zp > 0)
        {
            ParticleArray[5].setMass(BASE_MASS + EXTRA_MASS*xp*zp);
            MassDisplayPos.addScaledVector(ParticleArray[5].Position, xp*zp);
        }
    }
    if (zp > 0)
    {
        ParticleArray[3].setMass(BASE_MASS + EXTRA_MASS*(1-xp)*zp);
        MassDisplayPos.addScaledVector(ParticleArray[3].Position, (1-xp)*zp);
    }
}

void PlatformDemo::Display()
{
    MassAggregateApplication::Display();

    glBegin(GL_LINES);
    glColor3f(0,0,1);
    for (unsigned i = 0; i < ROD_COUNT; i++)
    {
        cyclone::Particle **particles = Rods[i].particle;
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

void PlatformDemo::Update()
{
    MassAggregateApplication::Update();

    UpdateAdditionalMass();
}

const char* PlatformDemo::GetTitle()
{
    return "Cyclone > Platform Demo";
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