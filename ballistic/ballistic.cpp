/*
 * The ballistic demo.
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

/**
 * The main demo class definition.
 */
class BallisticDemo : public Application
{
    enum ShotType
		{	UNUSED			 = 0
		,	PISTOL
		,	ARTILLERY
		,	FIREBALL
		,	LASER
		};

    // Holds a single ammunition round record.
    struct AmmoRound {
        cyclone::Particle	particle;
        ShotType			type;
        unsigned			startTime;

        /** Draws the round. */
        void render()
        {
            cyclone::Vector3 position = particle.Position;

            glColor3f(0, 0, 0);
            glPushMatrix();
            glTranslatef(position.x, position.y, position.z);
            glutSolidSphere(0.3f, 5, 4);
            glPopMatrix();

            glColor3f(0.75, 0.75, 0.75);
            glPushMatrix();
            glTranslatef(position.x, 0, position.z);
            glScalef(1.0f, 0.1f, 1.0f);
            glutSolidSphere(0.6f, 5, 4);
            glPopMatrix();
        }
    };

	const static unsigned	ammoRounds			= 16;	// Holds the maximum number of rounds that can be fired.
	AmmoRound				ammo[ammoRounds];	// Holds the particle data.
	ShotType				currentShotType;	// Holds the current shot type. 

	void					fire				();	// Dispatches a round.
public:
							BallisticDemo		();										// Creates a new demo object.
	
	virtual const char*		getTitle			()										{ return "Cyclone > Ballistic Demo"; }
	virtual void			update				();										// Update the particle positions.
	virtual void			display				();										// Display the particle positions.
	virtual void			mouse				(int button, int state, int x, int y);	// Handle a mouse click.
	virtual void			key					(unsigned char key);					// Handle a keypress. 
};

// Method definitions
BallisticDemo::BallisticDemo()
	: currentShotType(LASER)
{
	// Make all shots unused
	for (AmmoRound *shot = ammo; shot < ammo+ammoRounds; shot++)
		shot->type = UNUSED;
}

void BallisticDemo::fire() {
	AmmoRound			* shot				= 0;	// Find the first available round.
	for (shot = ammo; shot < ammo + ammoRounds; shot++)
		if (shot->type == UNUSED) 
			break;
	
	
	if (shot >= ammo + ammoRounds)	// If we didn't find a round, then exit - we can't fire. 
		return;

	switch(currentShotType)	{ // Set the properties of the particle
    case PISTOL:
        shot->particle.setMass(2.0f); // 2.0kg
        shot->particle.Velocity		= {0.0f, 0.0f, 35.0f}; // 35m/s
        shot->particle.Acceleration	= {0.0f, -1.0f, 0.0f};
        shot->particle.Damping		= 0.99f;
        break;

    case ARTILLERY:
        shot->particle.setMass(200.0f); // 200.0kg
        shot->particle.Velocity		= {0.0f, 30.0f, 40.0f}; // 50m/s
        shot->particle.Acceleration	= {0.0f, -20.0f, 0.0f};
        shot->particle.Damping		= 0.99f;
        break;

    case FIREBALL:
        shot->particle.setMass(1.0f);					// 1.0kg - mostly blast damage
        shot->particle.Velocity		= {0.0f, 0.0f, 10.0f};	// 5m/s
        shot->particle.Acceleration	= {0.0f, 0.6f, 0.0f };		// Floats up
        shot->particle.Damping		= (0.9f);
        break;						
									
    case LASER:						
        							
        shot->particle.setMass(0.1f);					// 0.1kg - almost no weight
        shot->particle.Velocity		= {0.0f, 0.0f, 100.0f	};	// 100m/s
        shot->particle.Acceleration	= {};		// No gravity
        shot->particle.Damping		= (0.99f);
        break; // Note that this is the kind of laser bolt seen in films, not a realistic laser beam!
    }

    // Set the data common to all particle types
	shot->particle.Position = {0.0f, 1.5f, 0.0f};
    shot->startTime = TimingData::get().lastFrameTimestamp;
    shot->type = currentShotType;

    // Clear the force accumulators
    shot->particle.clearAccumulator();
}

void BallisticDemo::update()
{
    // Find the duration of the last frame in seconds
    float duration = (float)TimingData::get().lastFrameDuration * 0.001f;
    if (duration <= 0.0f) return;

    // Update the physics of each particle in turn
    for (AmmoRound *shot = ammo; shot < ammo+ammoRounds; shot++)
    {
        if (shot->type != UNUSED)
        {
            // Run the physics
            shot->particle.integrate(duration);

            // Check if the particle is now invalid
            if (shot->particle.Position.y < 0.0f ||
                shot->startTime+5000 < TimingData::get().lastFrameTimestamp ||
                shot->particle.Position.z > 200.0f)
            {
                // We simply set the shot type to be unused, so the
                // memory it occupies can be reused by another shot.
                shot->type = UNUSED;
            }
        }
    }

    Application::update();
}

void BallisticDemo::display()
{
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

    for (AmmoRound *shot = ammo; shot < ammo+ammoRounds; shot++)	// Render each particle in turn
        if (shot->type != UNUSED)
            shot->render();

    // Render the description
    glColor3f(0.0f, 0.0f, 0.0f);
    renderText(10.0f, 34.0f, "Click: Fire\n1-4: Select Ammo");

    switch(currentShotType) {	// Render the name of the current shot type
    case PISTOL		: renderText(10.0f, 10.0f, "Current Ammo: Pistol"		); break;
    case ARTILLERY	: renderText(10.0f, 10.0f, "Current Ammo: Artillery"	); break;
    case FIREBALL	: renderText(10.0f, 10.0f, "Current Ammo: Fireball"		); break;
    case LASER		: renderText(10.0f, 10.0f, "Current Ammo: Laser"		); break;
    }
}

void BallisticDemo::mouse(int button, int state, int x, int y)
{
    // Fire the current weapon.
    if (state == GLUT_DOWN) 
		fire();
}

void BallisticDemo::key(unsigned char key)
{
    switch(key) {
    case '1': currentShotType = PISTOL		; break;
    case '2': currentShotType = ARTILLERY	; break;
    case '3': currentShotType = FIREBALL	; break;
    case '4': currentShotType = LASER		; break;
    }
}

/**
 * Called by the common demo framework to create an application
 * object (with new) and return a pointer.
 */
Application* getApplication()
{
    return new BallisticDemo();
}