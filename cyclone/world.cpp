// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include "world.h"

using namespace cyclone;

void									World::StartFrame				()									{
	BodyRegistration							* reg							= FirstBody;
	while (reg) {
		// Remove all forces from the accumulator
		reg->Body->clearAccumulators();
		reg->Body->CalculateDerivedData();
		reg										= reg->Next;	// Get the next registration
	}
}

uint32_t								World::GenerateContacts			()									{
	uint32_t									limit							= MaxContacts;
	Contact										* nextContact					= Contacts;

	ContactGenRegistration						* reg							= FirstContactGen;
	while (reg) {
		uint32_t									used							= reg->Generator->AddContact(nextContact, limit);
		limit									-= used;
		nextContact								+= used;
		if (limit <= 0)		// We've run out of contacts to fill. This means we're missing contacts.
			break;
		reg										= reg->Next;
	}
	return MaxContacts - limit;	// Return the number of contacts used.
}

void									World::RunPhysics				(double duration)					{
	//registry.UpdateForces(duration);	// First apply the force generators
	// Then integrate the objects
	BodyRegistration							* reg							= FirstBody;
	while (reg) {
		reg->Body->Integrate(duration);	// Remove all forces from the accumulator
		reg										= reg->Next;	// Get the next registration
	}
	uint32_t									usedContacts					= GenerateContacts();	// Generate contacts
	// And process them
	if (CalculateIterations) 
		Resolver.setIterations(usedContacts * 4);
	Resolver.resolveContacts(Contacts, usedContacts, duration);
}
