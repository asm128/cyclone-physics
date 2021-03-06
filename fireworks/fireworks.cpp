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
	uint32_t							Type								;	// Fireworks have an integer type, used for firework rules. 
	double								Age									;	// The age of a firework determines when it detonates. Age gradually decreases, when it passes zero the firework delivers its payload. Think of age as fuse-left.
	
	// Updates the firework by the given duration of time. Returns true if the firework has reached the end of its life and needs to be removed.
	bool								Update								(double duration)																																	{
		Integrate(duration);	// Update our physical state
		Age									-= duration;	// We work backwards from our age to zero.
		return (Age < 0) || (Position.y < 0);
	}
};

// Firework rules control the length of a firework's fuse and the particles it should evolve into.
struct FireworkRule {
	uint32_t							Type;			// The type of firework that is managed by this rule. 
	double								MinAge;			// The minimum length of the fuse. 
	double								MaxAge;			// The maximum legnth of the fuse. 
	::cyclone::Vector3					MinVelocity;	// The minimum relative velocity of this firework. 
	::cyclone::Vector3					MaxVelocity;	// The maximum relative velocity of this firework. 
	double								Damping;		// The damping of this firework type. 

	// The payload is the new firework type to create when this firework's fuse is over.
	struct Payload {
		uint32_t							Type;	// The type of the new particle to create. 
		uint32_t							Count;	// The number of particles in this payload.
		
		// Sets the payload properties in one go.
		void								Set								(uint32_t type, uint32_t count)																														{
			Type								= type;
			Count								= count;
		}
	};

	uint32_t							PayloadCount						= {};	// The number of payloads for this firework type.
	Payload								* Payloads							= 0;	// The set of payloads. 

										~FireworkRule						()																																					{
		if (Payloads) 
			delete[] Payloads;
	}
	void								Init								(uint32_t payloadCount)																																{
		PayloadCount						= payloadCount;
		Payloads							= new Payload[payloadCount];
	}
	// Set all the rule parameters in one go.
	void								SetParameters						(uint32_t type, double minAge, double maxAge, const ::cyclone::Vector3 &minVelocity, const ::cyclone::Vector3 &maxVelocity, double damping)			{
		Type								= type;
		MinAge								= minAge;
		MaxAge								= maxAge;
		MinVelocity							= minVelocity;
		MaxVelocity							= maxVelocity;
		Damping								= damping;
	}

	// Creates a new firework of this type and writes it into the gi	ven instance. The optional parent firework is used to base position and velocity on.
	void								create								(Firework *firework, const Firework *parent = NULL)																							const	{
		firework->Type						= Type;
		firework->Age						= crandom.RandomReal(MinAge, MaxAge);

		::cyclone::Vector3						vel									= {};
		if (parent) {	// The position and velocity are based on the parent.
			firework->Position					= (parent->Position);
			vel									+= parent->Velocity;
		}
		else {
			cyclone::Vector3						start;
			int										x									= (int)crandom.RandomInt(3) - 1;
			start.x								= 5.0f * (double)x;
			firework->Position					= (start);
		}

		vel									+= crandom.RandomVector(MinVelocity, MaxVelocity);
		firework->Velocity					= (vel);

		// We use a mass of one in all cases (no point having fireworks with different masses, since they are only under the influence of gravity).
		firework->SetMass(1);
		firework->Damping					= Damping;
		firework->Acceleration				= (cyclone::Vector3::GRAVITY);
		firework->AccumulatedForce			= {};
	}
};

// The main demo class definition.
class FireworksDemo : public Application {
	static	const uint32_t				MaxFireworks						= 1024;														// Holds the maximum number of fireworks that can be in use.
	static	const uint32_t				RuleCount							= 9;														// And the number of rules.
			Firework					Fireworks		[MaxFireworks]		= {};														// Holds the firework data.
			uint32_t					NextFirework						= 0;														// Holds the index of the next firework slot to use.
			FireworkRule				Rules			[RuleCount]			= {};														// Holds the set of rules.

			void						Create								(uint32_t type, const Firework *parent = NULL);				// Dispatches a firework from the origin.
			void						Create								(uint32_t type, uint32_t number, const Firework *parent);	// Dispatches the given number of fireworks from the given parent.
			void						InitFireworkRules					();															// Creates the rules.

public:
										FireworksDemo						()															{ InitFireworkRules();					}	// Create the firework types

	virtual	const char*					GetTitle							()															{ return "Cyclone > Fireworks Demo";	}
	virtual	void						InitGraphics						();															// Sets up the graphic rendering. 
	virtual	void						Update								();															// Update the particle positions. 
	virtual	void						Display								();															// Display the particle positions.
	virtual	void						Key									(unsigned char key);										// Handle a keypress.

	virtual	void						Mouse								(int, int, int, int)										{}	// Called when GLUT detects a mouse button press.
	virtual	void						MouseDrag							(int, int)													{}	// Called when GLUT detects a mouse drag.
};


void								FireworksDemo::InitGraphics			()															{
	Application::InitGraphics();			// Call the superclass
	glClearColor(0.0f, 0.0f, 0.1f, 1.0f);	// But override the clear color
}

void								FireworksDemo::Create				(uint32_t type, const Firework *parent)						{
	FireworkRule							* rule								= Rules + (type - 1);	// Get the rule needed to create this firework
	rule->create(Fireworks+NextFirework, parent);	// Create the firework
	NextFirework						= (NextFirework + 1) % MaxFireworks;	// Increment the index for the next firework
}

void								FireworksDemo::Create				(uint32_t type, uint32_t number, const Firework *parent)	{
	for (uint32_t i = 0; i < number; i++)
		Create(type, parent);
}

void								FireworksDemo::Update				()															{
	double									duration							= TimingData::get().LastFrameDuration * 0.001;	// Find the duration of the last frame in seconds
	if (duration <= 0.0) 
		return;

	for (Firework *firework = Fireworks; firework < Fireworks + MaxFireworks; ++firework) {
		if (firework->Type > 0)	{	// Check if we need to process this firework.
			if (firework->Update(duration))	{	// Does it need removing?
				FireworkRule							* rule								= Rules + (firework->Type - 1);	// Find the appropriate rule
				firework->Type						= 0;	// Delete the current firework (this doesn't affect its position and velocity for passing to the create function, just whether or not it is processed for rendering or physics.

				// Add the payload
				for (uint32_t i = 0; i < rule->PayloadCount; i++) {
					FireworkRule::Payload					* payload							= rule->Payloads + i;
					Create(payload->Type, payload->Count, firework);
				}
			}
		}
	}
	Application::Update();
}

void								FireworksDemo::Display				()															{
	const static double						size								= 0.1f;

	// Clear the viewport and set the camera direction
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();
	gluLookAt(0.0, 4.0, 10.0,  0.0, 4.0, 0.0,  0.0, 1.0, 0.0);

	// Render each firework in turn
	glBegin(GL_QUADS);
	for (Firework *firework = Fireworks; firework < Fireworks + MaxFireworks; firework++)
		if (firework->Type > 0) {	// Check if we need to process this firework.
			switch (firework->Type) {
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

			const cyclone::Vector3	& pos = firework->Position;
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
	glEnd();
}

void								FireworksDemo::Key					(unsigned char key)											{
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


void								FireworksDemo::InitFireworkRules	() {
	// Go through the firework types and create their rules.
	Rules[0].Init(2);
	Rules[0].SetParameters(
	    1,				// type
	    0.5f, 1.4f,		// age range
	    {-5, 25, -5},	// min velocity
	    {5,  28,  5},	// max velocity
	    0.1				// damping
	    );
	Rules[0].Payloads[0].Set(3, 5);
	Rules[0].Payloads[1].Set(5, 5);

	Rules[1].Init(1);
	Rules[1].SetParameters(
	    2,				// type
	    0.5f, 1.0f,		// age range
	    {-5, 10, -5},	// min velocity
	    { 5, 20,  5},	// max velocity
	    0.8				// damping
	    );
	Rules[1].Payloads[0].Set(4, 2);

	Rules[2].Init(0);
	Rules[2].SetParameters(
	    3,				// type
	    0.5f, 1.5f,		// age range
	    {-5, -5, -5},	// min velocity
	    { 5,  5,  5},	// max velocity
	    0.1				// damping
	    );

	Rules[3].Init(0);
	Rules[3].SetParameters(
	    4,				// type
	    0.25f, 0.5f,	// age range
	    {-20, 5, -5},	// min velocity
	    { 20, 5,  5},	// max velocity
	    0.2				// damping
	    );

	Rules[4].Init(1);
	Rules[4].SetParameters(
	    5,				// type
	    0.5f, 1.0f,		// age range
	    {-20, 2, -5},	// min velocity
	    { 20, 18, 5},	// max velocity
	    0.01			// damping
	    );
	Rules[4].Payloads[0].Set(3, 5);

	Rules[5].Init(0);
	Rules[5].SetParameters(
	    6,				// type
	    3, 5,			// age range
	    {-5, 5, -5},	// min velocity
	    { 5, 10, 5},	// max velocity
	    0.95			// damping
	    );

	Rules[6].Init(1);
	Rules[6].SetParameters(
	    7,				// type
	    4, 5,			// age range
	    {-5, 50, -5},	// min velocity
	    { 5, 60,  5},	// max velocity
	    0.01			// damping
	    );
	Rules[6].Payloads[0].Set(8, 10);

	Rules[7].Init(0);
	Rules[7].SetParameters(
	    8,				// type
	    0.25f, 0.5f,	// age range
	    {-1, -1, -1},	// min velocity
	    { 1,  1,  1},	// max velocity
	    0.01			// damping
	    );

	Rules[8].Init(0);
	Rules[8].SetParameters(
	    9,				// type
	    3, 5,			// age range
	    {-15, 10, -5},	// min velocity
	    { 15, 15,  5},	// max velocity
	    0.95			// damping
	    );
}

Application* getApplication() { return new FireworksDemo(); }	// Called by the common demo framework to create an application object (with new) and return a pointer.
