// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include "cyclone.h"
#include "ogl_headers.h"
#include "app.h"
#include "timing.h"

#include <stdio.h>

static cyclone::Random crandom;

// Fireworks are particles, with additional data for rendering and evolution.
class Firework : public cyclone::Particle {
public:
	uint32_t					type;	// Fireworks have an integer type, used for firework rules. 
	cyclone::real				age;	// The age of a firework determines when it detonates. Age gradually decreases, when it passes zero the firework delivers its payload. Think of age as fuse-left.
	
	// Updates the firework by the given duration of time. Returns true if the firework has reached the end of its life and needs to be removed.
	bool						Update							(cyclone::real duration)	{
		integrate(duration);	// Update our physical state
		age							-= duration;	// We work backwards from our age to zero.
		return (age < 0) || (Position.y < 0);
	}
};

// Firework rules control the length of a firework's fuse and the
// particles it should evolve into.
struct FireworkRule {
    uint32_t					type;			// The type of firework that is managed by this rule. 
    cyclone::real				minAge;			// The minimum length of the fuse. 
    cyclone::real				maxAge;			// The maximum legnth of the fuse. 
    cyclone::Vector3			minVelocity;	// The minimum relative velocity of this firework. 
    cyclone::Vector3			maxVelocity;	// The maximum relative velocity of this firework. 
    cyclone::real				damping;		// The damping of this firework type. 

    // The payload is the new firework type to create when this firework's fuse is over.
    struct Payload {
		uint32_t					type;	// The type of the new particle to create. 
		uint32_t					count;	// The number of particles in this payload.
		
		// Sets the payload properties in one go.
		void						set					(unsigned type, unsigned count)		{
			Payload::type				= type;
			Payload::count				= count;
		}
	};

	uint32_t					payloadCount			= {};	// The number of payloads for this firework type.
	Payload						* payloads				= 0;	// The set of payloads. 

	void						init					(unsigned payloadCount)				{
		FireworkRule::payloadCount	= payloadCount;
		payloads					= new Payload[payloadCount];
	}
	
								~FireworkRule			()									{
		if (payloads) 
			delete[] payloads;
    }

    /**
     * Set all the rule parameters in one go.
     */
    void setParameters(unsigned type, cyclone::real minAge, cyclone::real maxAge,
        const cyclone::Vector3 &minVelocity, const cyclone::Vector3 &maxVelocity,
        cyclone::real damping)
    {
        FireworkRule::type = type;
        FireworkRule::minAge = minAge;
        FireworkRule::maxAge = maxAge;
        FireworkRule::minVelocity = minVelocity;
        FireworkRule::maxVelocity = maxVelocity;
        FireworkRule::damping = damping;
    }

    /**
     * Creates a new firework of this type and writes it into the given
     * instance. The optional parent firework is used to base position
     * and velocity on.
     */
    void create(Firework *firework, const Firework *parent = NULL) const
    {
        firework->type = type;
        firework->age = crandom.randomReal(minAge, maxAge);

        cyclone::Vector3 vel;
        if (parent) {
            // The position and velocity are based on the parent.
            firework->Position = (parent->Position);
            vel += parent->Velocity;
        }
        else
        {
            cyclone::Vector3 start;
            int x = (int)crandom.randomInt(3) - 1;
            start.x = 5.0f * cyclone::real(x);
            firework->Position = (start);
        }

        vel += crandom.randomVector(minVelocity, maxVelocity);
        firework->Velocity = (vel);

        // We use a mass of one in all cases (no point having fireworks
        // with different masses, since they are only under the influence
        // of gravity).
        firework->setMass(1);

        firework->Damping = (damping);

        firework->Acceleration = (cyclone::Vector3::GRAVITY);

        firework->clearAccumulator();
    }
};

/**
 * The main demo class definition.
 */
class FireworksDemo : public Application {
	static const uint32_t	maxFireworks					= 1024;														// Holds the maximum number of fireworks that can be in use.
	static const uint32_t	ruleCount						= 9;														// And the number of rules.
	Firework				fireworks		[maxFireworks];																// Holds the firework data.
	unsigned				nextFirework;																				// Holds the index of the next firework slot to use.
	FireworkRule			rules			[ruleCount];																// Holds the set of rules.

	void					Create							(unsigned type, const Firework *parent=NULL);				// Dispatches a firework from the origin.
	void					Create							(unsigned type, unsigned number, const Firework *parent);	// Dispatches the given number of fireworks from the given parent.
	void					InitFireworkRules				();															// Creates the rules.

public:
							FireworksDemo					();

	virtual void			InitGraphics					();															// Sets up the graphic rendering. 
	virtual const char*		GetTitle						();															// Returns the window title for the demo. 
	virtual void			Update							();															// Update the particle positions. 
	virtual void			Display							();															// Display the particle positions.
	virtual void			Key								(unsigned char key);										// Handle a keypress.
};

// Method definitions
FireworksDemo::FireworksDemo()
: nextFirework(0)
{
    // Make all shots unused
    for (Firework *firework = fireworks;
         firework < fireworks+maxFireworks;
         firework++)
    {
        firework->type = 0;
    }

    // Create the firework types
    InitFireworkRules();
}
void FireworksDemo::InitFireworkRules()
{
    // Go through the firework types and create their rules.
    rules[0].init(2);
    rules[0].setParameters(
        1, // type
        0.5f, 1.4f, // age range
        cyclone::Vector3(-5, 25, -5), // min velocity
        cyclone::Vector3(5, 28, 5), // max velocity
        0.1 // damping
        );
    rules[0].payloads[0].set(3, 5);
    rules[0].payloads[1].set(5, 5);

    rules[1].init(1);
    rules[1].setParameters(
        2, // type
        0.5f, 1.0f, // age range
        cyclone::Vector3(-5, 10, -5), // min velocity
        cyclone::Vector3(5, 20, 5), // max velocity
        0.8 // damping
        );
    rules[1].payloads[0].set(4, 2);

    rules[2].init(0);
    rules[2].setParameters(
        3, // type
        0.5f, 1.5f, // age range
        cyclone::Vector3(-5, -5, -5), // min velocity
        cyclone::Vector3(5, 5, 5), // max velocity
        0.1 // damping
        );

    rules[3].init(0);
    rules[3].setParameters(
        4, // type
        0.25f, 0.5f, // age range
        cyclone::Vector3(-20, 5, -5), // min velocity
        cyclone::Vector3(20, 5, 5), // max velocity
        0.2 // damping
        );

    rules[4].init(1);
    rules[4].setParameters(
        5, // type
        0.5f, 1.0f, // age range
        cyclone::Vector3(-20, 2, -5), // min velocity
        cyclone::Vector3(20, 18, 5), // max velocity
        0.01 // damping
        );
    rules[4].payloads[0].set(3, 5);

    rules[5].init(0);
    rules[5].setParameters(
        6, // type
        3, 5, // age range
        cyclone::Vector3(-5, 5, -5), // min velocity
        cyclone::Vector3(5, 10, 5), // max velocity
        0.95 // damping
        );

    rules[6].init(1);
    rules[6].setParameters(
        7, // type
        4, 5, // age range
        cyclone::Vector3(-5, 50, -5), // min velocity
        cyclone::Vector3(5, 60, 5), // max velocity
        0.01 // damping
        );
    rules[6].payloads[0].set(8, 10);

    rules[7].init(0);
    rules[7].setParameters(
        8, // type
        0.25f, 0.5f, // age range
        cyclone::Vector3(-1, -1, -1), // min velocity
        cyclone::Vector3(1, 1, 1), // max velocity
        0.01 // damping
        );

    rules[8].init(0);
    rules[8].setParameters(
        9, // type
        3, 5, // age range
        cyclone::Vector3(-15, 10, -5), // min velocity
        cyclone::Vector3(15, 15, 5), // max velocity
        0.95 // damping
        );
    // ... and so on for other firework types ...
}

void FireworksDemo::InitGraphics()
{
    // Call the superclass
    Application::InitGraphics();

    // But override the clear color
    glClearColor(0.0f, 0.0f, 0.1f, 1.0f);
}

const char* FireworksDemo::GetTitle()
{
    return "Cyclone > Fireworks Demo";
}

void FireworksDemo::Create(unsigned type, const Firework *parent)
{
    FireworkRule *rule = rules + (type - 1);	// Get the rule needed to create this firework
    rule->create(fireworks+nextFirework, parent);	// Create the firework
    nextFirework = (nextFirework + 1) % maxFireworks;	// Increment the index for the next firework
}

void FireworksDemo::Create(unsigned type, unsigned number, const Firework *parent)
{
    for (unsigned i = 0; i < number; i++)
        Create(type, parent);
}

void FireworksDemo::Update()
{
    // Find the duration of the last frame in seconds
    float duration = (float)TimingData::get().lastFrameDuration * 0.001f;
    if (duration <= 0.0f) return;

    for (Firework *firework = fireworks;
         firework < fireworks+maxFireworks;
         firework++)
    {
        
        if (firework->type > 0)	{	// Check if we need to process this firework.
            if (firework->Update(duration))	{	// Does it need removing?
                FireworkRule *rule = rules + (firework->type-1);	// Find the appropriate rule

                // Delete the current firework (this doesn't affect its position and velocity for passing to the create function, just whether or not it is processed for rendering or physics.
                firework->type = 0;

                // Add the payload
                for (unsigned i = 0; i < rule->payloadCount; i++) {
                    FireworkRule::Payload * payload = rule->payloads + i;
                    Create(payload->type, payload->count, firework);
                }
            }
        }
    }

    Application::Update();
}

void FireworksDemo::Display()
{
    const static cyclone::real size = 0.1f;

    // Clear the viewport and set the camera direction
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    gluLookAt(0.0, 4.0, 10.0,  0.0, 4.0, 0.0,  0.0, 1.0, 0.0);

    // Render each firework in turn
    glBegin(GL_QUADS);
    for (Firework *firework = fireworks;
        firework < fireworks+maxFireworks;
        firework++)
    {
        // Check if we need to process this firework.
        if (firework->type > 0) {
            switch (firework->type) {
            case 1: glColor3f(1,0,0);			break;
            case 2: glColor3f(1,0.5f,0);		break;
            case 3: glColor3f(1,1,0);			break;
            case 4: glColor3f(0,1,0);			break;
            case 5: glColor3f(0,1,1);			break;
            case 6: glColor3f(0.4f,0.4f,1);		break;
            case 7: glColor3f(1,0,1);			break;
            case 8: glColor3f(1,1,1);			break;
            case 9: glColor3f(1,0.5f,0.5f);		break;
            };

            const cyclone::Vector3 &pos = firework->Position;
            glVertex3f(pos.x-size, pos.y-size, pos.z);
            glVertex3f(pos.x+size, pos.y-size, pos.z);
            glVertex3f(pos.x+size, pos.y+size, pos.z);
            glVertex3f(pos.x-size, pos.y+size, pos.z);

            // Render the firework's reflection
            glVertex3f(pos.x-size, -pos.y-size, pos.z);
            glVertex3f(pos.x+size, -pos.y-size, pos.z);
            glVertex3f(pos.x+size, -pos.y+size, pos.z);
            glVertex3f(pos.x-size, -pos.y+size, pos.z);
        }
    }
    glEnd();
}

void FireworksDemo::Key(unsigned char key)
{
    switch (key) {
    case '1': Create(1, 1, NULL); break;
    case '2': Create(2, 1, NULL); break;
    case '3': Create(3, 1, NULL); break;
    case '4': Create(4, 1, NULL); break;
    case '5': Create(5, 1, NULL); break;
    case '6': Create(6, 1, NULL); break;
    case '7': Create(7, 1, NULL); break;
    case '8': Create(8, 1, NULL); break;
    case '9': Create(9, 1, NULL); break;
    }
}


// Called by the common demo framework to create an application object (with new) and return a pointer.
Application* getApplication()
{
    return new FireworksDemo();
}
