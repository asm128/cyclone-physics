// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include "pworld.h"

#include <cstddef>

using namespace cyclone;

									ParticleWorld::ParticleWorld		(uint32_t maxContacts, uint32_t iterations)
	: Resolver		(iterations	)
	, MaxContacts	(maxContacts)
{
	Contacts							= new ParticleContact[maxContacts];
	CalculateIterations					= (iterations == 0);
}

									ParticleWorld::~ParticleWorld		()																		{
	if(Contacts)
		delete[] Contacts;
}

void								ParticleWorld::StartFrame			()																		{
	for (TParticles::iterator p = Particles.begin(); p != Particles.end(); ++p)
		(*p)->AccumulatedForce				= {};	// Remove all forces from the accumulator
}

uint32_t							ParticleWorld::GenerateContacts		()																		{
	uint32_t								limit								= MaxContacts;
	ParticleContact							* nextContact						= Contacts;
	for (TContactGenerators::iterator g = ContactGenerators.begin(); g != ContactGenerators.end(); ++g) {
		uint32_t								used								= (*g)->AddContact(nextContact, limit);
		limit								-= used;
		nextContact							+= used;

		if (limit <= 0)		// We've run out of contacts to fill. This means we're missing contacts.
			break;
	}
	return MaxContacts - limit;	// Return the number of contacts used.
}

void								ParticleWorld::Integrate			(double duration)														{
	for (TParticles::iterator p = Particles.begin(); p != Particles.end(); ++p)
		(*p)->Integrate(duration);		// Remove all forces from the accumulator
}

void								ParticleWorld::RunPhysics			(double duration)														{
	ForceRegistry.UpdateForces	(duration);		// First apply the force generators
	Integrate					(duration);		// Then integrate the objects
	uint32_t								usedContacts						= GenerateContacts();	// Generate contacts
	if (usedContacts) {// And process them
		if (CalculateIterations) 
			Resolver.SetIterations(usedContacts * 2);
		Resolver.ResolveContacts(Contacts, usedContacts, duration);
	}
}

uint32_t							GroundContacts::AddContact			(cyclone::ParticleContact *contact, uint32_t limit)				const	{
	uint32_t								count								= 0;
	for (cyclone::ParticleWorld::TParticles::iterator p = Particles->begin(); p != Particles->end(); ++p) {
		double									y									= (*p)->Position.y;
		if (y < 0.0f) {
			contact->ContactNormal				= cyclone::Vector3::UP;
			contact->Particle[0]				= *p;
			contact->Particle[1]				= NULL;
			contact->Penetration				= -y;
			contact->Restitution				= 0.2f;
			++contact;
			++count;
		}
		if (count >= limit) 
			return count;
	}
	return count;
}