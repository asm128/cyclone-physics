// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include "world.h"

#include <cstdlib>

using namespace cyclone;

World::World(uint32_t maxContacts, uint32_t iterations)
	: FirstBody			(NULL)
	, Resolver			(iterations)
	, FirstContactGen	(NULL)
	, MaxContacts			(maxContacts)
{
	Contacts				= new Contact[maxContacts];
	calculateIterations		= (iterations == 0);
}

void World::startFrame() {
	BodyRegistration *reg = FirstBody;
	while (reg) {
		// Remove all forces from the accumulator
		reg->body->clearAccumulators();
		reg->body->CalculateDerivedData();
		reg				= reg->next;	// Get the next registration
	}
}

uint32_t World::GenerateContacts()
{
	uint32_t					limit			= MaxContacts;
	Contact						* nextContact	= Contacts;

	ContactGenRegistration		* reg			= FirstContactGen;
	while (reg) {
		uint32_t					used			= reg->gen->AddContact(nextContact, limit);
		limit					-= used;
		nextContact				+= used;

		// We've run out of contacts to fill. This means we're missing contacts.
		if (limit <= 0) 
			break;

		reg						= reg->next;
	}
	return MaxContacts - limit;	// Return the number of contacts used.
}

void World::runPhysics(double duration)
{
	//registry.UpdateForces(duration);	// First apply the force generators

	// Then integrate the objects
	BodyRegistration *reg = FirstBody;
	while (reg) {
		reg->body->integrate(duration);	// Remove all forces from the accumulator
		reg = reg->next;	// Get the next registration
	}

	
	uint32_t usedContacts = GenerateContacts();	// Generate contacts

	// And process them
	if (calculateIterations) 
		Resolver.setIterations(usedContacts * 4);
	Resolver.resolveContacts(Contacts, usedContacts, duration);
}
