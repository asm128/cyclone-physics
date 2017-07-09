// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include "plinks.h"

using namespace cyclone;

double								ParticleLink::CurrentLength				()																const	{
	Vector3									relativePos								= Particle[0]->Position - Particle[1]->Position;
	return relativePos.magnitude();
}

uint32_t							ParticleCable::AddContact				(ParticleContact *contact, uint32_t limit)						const	{
	double									length									= CurrentLength();	// Find the length of the cable
	if (length < MaxLength)	// Check if we're over-extended
		return 0;

	// Otherwise return the contact
	contact->Particle[0]				= Particle[0];
	contact->Particle[1]				= Particle[1];

	// Calculate the normal
	Vector3									normal									= Particle[1]->Position - Particle[0]->Position;
	normal.normalise();
	contact->ContactNormal				= normal;
	contact->Penetration				= length - MaxLength;
	contact->Restitution				= Restitution;

	return 1;
}

uint32_t							ParticleRod::AddContact					(ParticleContact *contact, uint32_t limit)						const	{
	double									currentLen								= CurrentLength();	// Find the length of the rod
	if (currentLen == Length)	// Check if we're over-extended
	    return 0;

	// Otherwise return the contact
	contact->Particle[0]				= Particle[0];
	contact->Particle[1]				= Particle[1];

	// Calculate the normal
	Vector3									normal									= Particle[1]->Position - Particle[0]->Position;
	normal.normalise();

	if (currentLen > Length) {	// The contact normal depends on whether we're extending or compressing
		contact->ContactNormal				= normal;
		contact->Penetration				= currentLen - Length;
	} else {
		contact->ContactNormal				= normal * -1;
		contact->Penetration				= Length - currentLen;
	}

	contact->Restitution				= 0;	// Always use zero restitution (no bounciness)
	return 1;
}

double								ParticleConstraint::CurrentLength		()																const	{
	Vector3									relativePos								= Particle->Position - Anchor;
	return relativePos.magnitude();
}

uint32_t							ParticleCableConstraint::AddContact		(ParticleContact *contact, uint32_t limit)						const	{
	double									length									= CurrentLength(); // Find the length of the cable
	if (length < MaxLength)	// Check if we're over-extended
		return 0;

	// Otherwise return the contact
	contact->Particle[0]				= Particle;
	contact->Particle[1]				= 0;

	// Calculate the normal
	Vector3									normal									= Anchor - Particle->Position;
	normal.normalise();
	contact->ContactNormal				= normal;

	contact->Penetration				= length - MaxLength;
	contact->Restitution				= Restitution;

	return 1;
}

uint32_t							ParticleRodConstraint::AddContact		(ParticleContact *contact, uint32_t limit)						const	{
	double									currentLen								= CurrentLength();	// Find the length of the rod
	if (currentLen == Length)	// Check if we're over-extended
		return 0;

	// Otherwise return the contact
	contact->Particle[0]				= Particle;
	contact->Particle[1]				= 0;

	// Calculate the normal
	Vector3									normal									= Anchor - Particle->Position;
	normal.normalise();

	if (currentLen > Length) {	// The contact normal depends on whether we're extending or compressing
		contact->ContactNormal				= normal;
		contact->Penetration				= currentLen - Length;
	} else {
		contact->ContactNormal				= normal * -1;
		contact->Penetration				= Length - currentLen;
	}
	contact->Restitution				= 0;	// Always use zero restitution (no bounciness)
	return 1;
}