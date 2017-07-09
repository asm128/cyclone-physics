// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include "plinks.h"

using namespace cyclone;

double								ParticleLink::currentLength				()																const	{
	Vector3									relativePos								= particle[0]->Position - particle[1]->Position;
	return relativePos.magnitude();
}

uint32_t							ParticleCable::AddContact				(ParticleContact *contact, uint32_t limit)						const	{
	double									length									= currentLength();	// Find the length of the cable
	if (length < maxLength)	// Check if we're over-extended
		return 0;

	// Otherwise return the contact
	contact->Particle[0]				= particle[0];
	contact->Particle[1]				= particle[1];

	// Calculate the normal
	Vector3									normal									= particle[1]->Position - particle[0]->Position;
	normal.normalise();
	contact->ContactNormal				= normal;
	contact->Penetration				= length - maxLength;
	contact->Restitution				= restitution;

	return 1;
}

uint32_t							ParticleRod::AddContact					(ParticleContact *contact, uint32_t limit)						const	{
	double									currentLen								= currentLength();	// Find the length of the rod
	if (currentLen == length)	// Check if we're over-extended
	    return 0;

	// Otherwise return the contact
	contact->Particle[0]				= particle[0];
	contact->Particle[1]				= particle[1];

	// Calculate the normal
	Vector3									normal									= particle[1]->Position - particle[0]->Position;
	normal.normalise();

	if (currentLen > length) {	// The contact normal depends on whether we're extending or compressing
		contact->ContactNormal				= normal;
		contact->Penetration				= currentLen - length;
	} else {
		contact->ContactNormal				= normal * -1;
		contact->Penetration				= length - currentLen;
	}

	contact->Restitution				= 0;	// Always use zero restitution (no bounciness)
	return 1;
}

double								ParticleConstraint::currentLength		()																const	{
	Vector3									relativePos								= particle->Position - anchor;
	return relativePos.magnitude();
}

uint32_t							ParticleCableConstraint::AddContact		(ParticleContact *contact, uint32_t limit)						const	{
	double									length									= currentLength(); // Find the length of the cable
	if (length < maxLength)	// Check if we're over-extended
		return 0;

	// Otherwise return the contact
	contact->Particle[0]				= particle;
	contact->Particle[1]				= 0;

	// Calculate the normal
	Vector3									normal									= anchor - particle->Position;
	normal.normalise();
	contact->ContactNormal				= normal;

	contact->Penetration				= length-maxLength;
	contact->Restitution				= restitution;

	return 1;
}

uint32_t							ParticleRodConstraint::AddContact		(ParticleContact *contact, uint32_t limit)						const	{
	double									currentLen								= currentLength();	// Find the length of the rod
	if (currentLen == length)	// Check if we're over-extended
		return 0;

	// Otherwise return the contact
	contact->Particle[0]				= particle;
	contact->Particle[1]				= 0;

	// Calculate the normal
	Vector3									normal									= anchor - particle->Position;
	normal.normalise();

	if (currentLen > length) {	// The contact normal depends on whether we're extending or compressing
		contact->ContactNormal				= normal;
		contact->Penetration				= currentLen - length;
	} else {
		contact->ContactNormal				= normal * -1;
		contact->Penetration				= length - currentLen;
	}
	contact->Restitution				= 0;	// Always use zero restitution (no bounciness)
	return 1;
}