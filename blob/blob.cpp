// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include "cyclone.h"
#include "ogl_headers.h"
#include "app.h"
#include "timing.h"

#include <stdio.h>
#include <cassert>

#define BLOB_COUNT		5
#define PLATFORM_COUNT	10
#define BLOB_RADIUS		0.4f


// Platforms are two dimensional: lines on which the particles can rest. Platforms are also contact generators for the physics.
class Platform : public cyclone::ParticleContactGenerator {
public:
			::cyclone::Vector3			Start							= {};
			::cyclone::Vector3			End								= {};
			::cyclone::Particle			* Particles						= 0;	// Holds a pointer to the particles we're checking for collisions with.

	virtual	uint32_t					AddContact						(cyclone::ParticleContact *contact, uint32_t limit) const;
};

uint32_t								Platform::AddContact			(cyclone::ParticleContact *contact, uint32_t limit) const {
	const static double restitution = 0.0f;

	uint32_t used = 0;
	for (uint32_t i = 0; i < BLOB_COUNT; ++i) {
		if (used >= limit) 
			break;

		// Check for penetration
		::cyclone::Vector3							toParticle						= Particles[i].Position - Start;
		::cyclone::Vector3							lineDirection					= End - Start;
		double										projected						= toParticle * lineDirection;
		double										platformSqLength				= lineDirection.squareMagnitude();
		if (projected <= 0) {	// The blob is nearest to the start point
			if (toParticle.squareMagnitude() < BLOB_RADIUS*BLOB_RADIUS) {	// We have a collision
				contact->ContactNormal					= toParticle.unit();
				contact->ContactNormal.z				= 0;
				contact->Restitution					= restitution;
				contact->Particle[0]					= Particles + i;
				contact->Particle[1]					= 0;
				contact->Penetration					= BLOB_RADIUS - toParticle.magnitude();
				++used;
				++contact;
			}
		}
		else if (projected >= platformSqLength) {
			// The blob is nearest to the end point
			toParticle								= Particles[i].Position - End;
			if (toParticle.squareMagnitude() < BLOB_RADIUS*BLOB_RADIUS) {	// We have a collision
				contact->ContactNormal					= toParticle.unit();
				contact->ContactNormal.z				= 0;
				contact->Restitution					= restitution;
				contact->Particle[0]					= Particles + i;
				contact->Particle[1]					= 0;
				contact->Penetration					= BLOB_RADIUS - toParticle.magnitude();
				++used;
				++contact;
			}
		}
		else {	// the blob is nearest to the middle.
			double										distanceToPlatform				= toParticle.squareMagnitude() - projected*projected / platformSqLength;
			if (distanceToPlatform < BLOB_RADIUS*BLOB_RADIUS) {	// We have a collision
				::cyclone::Vector3							closestPoint					= Start + lineDirection*(projected/platformSqLength);
				contact->ContactNormal					= (Particles[i].Position-closestPoint).unit();
				contact->ContactNormal.z				= 0;
				contact->Restitution					= restitution;
				contact->Particle[0]					= Particles + i;
				contact->Particle[1]					= 0;
				contact->Penetration					= BLOB_RADIUS - real_sqrt(distanceToPlatform);
				++used;
				++contact;
			}
		}
	}
	return used;
}

// A force generator for proximal attraction.
class BlobForceGenerator : public cyclone::ParticleForceGenerator {
public:
			::cyclone::Particle			* Particles						= 0; // Holds a pointer to the particles we might be attracting.
			double						MaxReplusion					= 0; // The maximum force used to push the particles apart.
			double						MaxAttraction					= 0; // The maximum force used to pull particles together.
			double						MinNaturalDistance				= 0; // The separation between particles where there is no force.
			double						MaxNaturalDistance				= 0; // The separation between particles where there is no force.
			double						FloatHead						= 0; // The force with which to float the head particle, if it is joined to others.
			uint32_t					MaxFloat						= 0; // The maximum number of particles in the blob before the head is floated at maximum force.
			double						MaxDistance						= 0; // The separation between particles after which they 'break' apart and there is no force.

	virtual	void						UpdateForce						(::cyclone::Particle *particle, double duration)				{
		uint32_t									joinCount						= 0;
		for (uint32_t i = 0; i < BLOB_COUNT; i++) {
			if (Particles + i == particle)	// Don't attract yourself
				continue;

			// Work out the separation distance
			cyclone::Vector3				separation				= Particles[i].Position - particle->Position;
			separation.z				= 0.0f;
			double							distance				= separation.magnitude();

			if (distance < MinNaturalDistance) {	// Use a repulsion force.
				distance					= 1.0f - distance / MinNaturalDistance;
				particle->AccumulatedForce	+= separation.unit() * (1.0f - distance) * MaxReplusion * -1.0f;
				joinCount++;
			}
			else if (distance > MaxNaturalDistance && distance < MaxDistance) {	// Use an attraction force.
				distance 
					= (distance		- MaxNaturalDistance) 
					/ (MaxDistance	- MaxNaturalDistance)
					;
				particle->AccumulatedForce			+= separation.unit() * distance * MaxAttraction;
				joinCount++;
			}
		}

		// If the particle is the head, and we've got a join count, then float it.
		if (particle == Particles && joinCount > 0 && MaxFloat > 0) {
			double									force						= (double(joinCount) / MaxFloat) * FloatHead;
			if (force > FloatHead) 
				force								= FloatHead;
			particle->AccumulatedForce			+= {0, force, 0};
		}
	}
};

// The main demo class definition.
class BlobDemo : public Application {
			cyclone::Particle			* Blobs				= 0;
			Platform					* Platforms			= 0;
			cyclone::ParticleWorld		World				;
			BlobForceGenerator			BlobForceGenerator	;
			float						AxisX				= 0;						// The control for the x-axis.
			float						AxisY				= 0;						// The control for the y-axis.
			
			void						Reset				();
public:
	virtual								~BlobDemo			()										{ if(Blobs)	delete Blobs;		}		
										BlobDemo			();

	virtual	void						Display				();						// Display the particles. 
	virtual	void						Update				();						// Update the particle positions. 

	virtual	const char*					GetTitle			()										{ return "Cyclone > Blob Demo";	}
	virtual	void						Mouse				(int button, int state, int x, int y)	{}	// Called when GLUT detects a mouse button press.
	virtual	void						MouseDrag			(int x, int y)							{}	// Called when GLUT detects a mouse drag.
	virtual	void						Key					(unsigned char key)						{
		switch(key) {
		case 'w': case 'W': AxisY				=  1.0;	break;
		case 's': case 'S': AxisY				= -1.0;	break;
		case 'a': case 'A': AxisX				= -1.0;	break;
		case 'd': case 'D': AxisX				=  1.0;	break;
		case 'r': case 'R': Reset();					break;
		}
	}
};

// Method definitions
BlobDemo::BlobDemo()
	: World(PLATFORM_COUNT+BLOB_COUNT, PLATFORM_COUNT)
{
	Blobs									= new cyclone::Particle[BLOB_COUNT];	// Create the blob storage
	cyclone::Random								r;

	// Create the force generator
	BlobForceGenerator.Particles			= Blobs;
	BlobForceGenerator.MaxAttraction		= 20.0f;
	BlobForceGenerator.MaxReplusion			= 10.0f;
	BlobForceGenerator.MinNaturalDistance	= BLOB_RADIUS*0.75f;
	BlobForceGenerator.MaxNaturalDistance	= BLOB_RADIUS*1.5f;
	BlobForceGenerator.MaxDistance			= BLOB_RADIUS * 2.5f;
	BlobForceGenerator.MaxFloat				= 2;
	BlobForceGenerator.FloatHead			= 8.0f;

	// Create the platforms
	Platforms								= new Platform[PLATFORM_COUNT];
	for (uint32_t i = 0; i < PLATFORM_COUNT; i++) {
		Platforms[i].Start						= 
			{ (i % 2) * 10.0f - 5.0f
			, i * 4.0f + ((i % 2) ? 0.0f : 2.0f)
			, 0
			};
		Platforms[i].Start.x					+= r.RandomBinomial(2.0f);
		Platforms[i].Start.y					+= r.RandomBinomial(2.0f);

		Platforms[i].End						= 
			{ (i % 2) * 10.0 + 5.0
			, i * 4.0 + ((i % 2) ? 2.0 : 0.0)
			, 0
			};
		Platforms[i].End.x						+= r.RandomBinomial(2.0f);
		Platforms[i].End.y						+= r.RandomBinomial(2.0f);

		// Make sure the platform knows which particles it should collide with.
		Platforms[i].Particles					= Blobs;
		World.ContactGenerators.push_back(Platforms + i);
	}

	// Create the blobs.
	Platform					* p			= Platforms + (PLATFORM_COUNT-2);
	double						fraction	= 1.0 / BLOB_COUNT;
	cyclone::Vector3			delta		= p->End - p->Start;
	for (uint32_t i = 0; i < BLOB_COUNT; i++) {
		uint32_t me = (i + BLOB_COUNT / 2) % BLOB_COUNT;
		Blobs[i].Position = (p->Start + delta * (me * 0.8 * fraction + 0.1) + cyclone::Vector3{0, 1.0 + r.RandomReal(), 0});

		Blobs[i].Velocity			= {};
		Blobs[i].Damping			= 0.2f;
		Blobs[i].Acceleration		= cyclone::Vector3::GRAVITY * 0.4f;
		Blobs[i].SetMass			(1.0f);
		Blobs[i].AccumulatedForce	= {};

		World.Particles.push_back(Blobs + i);
		World.ForceRegistry.Registrations.push_back({Blobs + i, &BlobForceGenerator});
	}
}

void BlobDemo::Reset() {
	cyclone::Random		r;
	Platform			* p				= Platforms + (PLATFORM_COUNT-2);
	double				fraction		= 1.0 / BLOB_COUNT;
	cyclone::Vector3	delta			= p->End - p->Start;
	for (uint32_t i = 0; i < BLOB_COUNT; i++) {
		uint32_t			me				= (i + BLOB_COUNT / 2) % BLOB_COUNT;
		Blobs[i].Position					= (p->Start + delta * (me * 0.8 * fraction + 0.1) + cyclone::Vector3{0, 1.0 + r.RandomReal(), 0});
		Blobs[i].Velocity					= {};
		Blobs[i].AccumulatedForce			= {};
	}
}


void BlobDemo::Display() {
	cyclone::Vector3 pos = Blobs[0].Position;

	// Clear the view port and set the camera direction
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();
	gluLookAt(pos.x, pos.y, 6.0,  pos.x, pos.y, 0.0,  0.0, 1.0, 0.0);

	glColor3f(0,0,0);


	glBegin(GL_LINES);
	glColor3f(0,0,1);
	for (uint32_t i = 0; i < PLATFORM_COUNT; i++) {
		const cyclone::Vector3 &p0 = Platforms[i].Start;
		const cyclone::Vector3 &p1 = Platforms[i].End;
		glVertex3f(p0.x, p0.y, p0.z);
		glVertex3f(p1.x, p1.y, p1.z);
	}
	glEnd();

	glColor3f(1,0,0);
	for (uint32_t i = 0; i < BLOB_COUNT; i++) {
		const cyclone::Vector3 &p = Blobs[i].Position;
		glPushMatrix();
		glTranslatef(p.x, p.y, p.z);
		glutSolidSphere(BLOB_RADIUS, 12, 12);
		glPopMatrix();
	}

	cyclone::Vector3 p = Blobs[0].Position;
	cyclone::Vector3 v = Blobs[0].Velocity * 0.05f;
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
	World.StartFrame();	// Clear accumulators

	// Find the duration of the last frame in seconds
	double duration = TimingData::get().LastFrameDuration * 0.001;
	if (duration <= 0.0f) 
		return;
	
	// Recenter the axes
	AxisX *= pow(0.1f, duration);
	AxisY *= pow(0.1f, duration);
	
	Blobs[0].AccumulatedForce	+= cyclone::Vector3{AxisX, AxisY, 0} * 10.0f;	// Move the controlled blob
	World.RunPhysics(duration);	// Run the simulation
	
	// Bring all the particles back to 2d
	cyclone::Vector3				position;
	for (uint32_t i = 0; i < BLOB_COUNT; ++i) {
		position					= Blobs[i].Position;
		position.z					= 0.0f;
		Blobs[i].Position			= position;
	}
	
	Application::Update();
}


	
Application* getApplication() { return new BlobDemo(); }	// Called by the common demo framework to create an application object (with new) and return a pointer.