// This file contains classes representing the connections between particles.
// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include "pcontacts.h"

#ifndef CYCLONE_PLINKS_H
#define CYCLONE_PLINKS_H

namespace cyclone {

	// Links connect two particles together, generating a contact if they violate the constraints of their link. 
	// It is used as a base class for cables and rods, and could be used as a base class for springs with a limit to their extension..
    class ParticleLink : public ParticleContactGenerator {
    public:
		Particle*							Particle	[2]				= {};	// Holds the pair of particles that are connected by this link.

		double								CurrentLength				()															const;	// Returns the current length of the link.
		// Geneates the contacts to keep this link from being violated. 
		// This class can only ever generate a single contact, so the pointer can be a pointer to a single element, the limit parameter is assumed to be at least one (zero isn't valid) and the return value is either 0, if the cable wasn't over-extended, or one if a contact was needed.
		// NB: This method is declared in the same way (as pure virtual) in the parent class, but is replicated here for documentation purposes.
		virtual uint32_t					AddContact					(ParticleContact *contact, uint32_t limit)					const	= 0;
	};

	// Cables link a pair of particles, generating a contact if they stray too far apart.
	class ParticleCable : public ParticleLink {
	public:
		double								MaxLength;					// Holds the maximum length of the cable.
		double								Restitution;				// Holds the restitution (bounciness) of the cable.

	public:
		virtual uint32_t					AddContact					(ParticleContact *contact, uint32_t limit)					const;	// Fills the given contact structure with the contact needed to keep the cable from over-extending.
	};

	// Rods link a pair of particles, generating a contact if they stray too far apart or too close.
	class ParticleRod : public ParticleLink {
	public:
		double								Length;	// Holds the length of the rod.

		virtual uint32_t					AddContact					(ParticleContact *contact, uint32_t limit)					const;	// Fills the given contact structure with the contact needed to keep the rod from extending or compressing.
	};

	// Constraints are just like links, except they connect a particle to an immovable anchor point.
	class ParticleConstraint : public ParticleContactGenerator {
	public:
		Particle							* Particle					= 0;	// Holds the particles connected by this constraint.
		Vector3								Anchor;	// The point to which the particle is anchored.

	protected:
		double								CurrentLength				()															const;	// Returns the current length of the link.

	public:
		// Geneates the contacts to keep this link from being violated. This class can only ever generate a single contact, so the pointer can be a pointer to a single element, 
		// the limit parameter is assumed to be at least one (zero isn't valid) and the return value is either 0, if the cable wasn't over-extended, or one if a contact was needed.
		// NB: This method is declared in the same way (as pure virtual) in the parent class, but is replicated here for documentation purposes.
		virtual uint32_t					AddContact					(ParticleContact *contact, uint32_t limit)					const	= 0;
	};

	// Cables link a particle to an anchor point, generating a contact if they stray too far apart.
	class ParticleCableConstraint : public ParticleConstraint {
	public:
		double								MaxLength;					// Holds the maximum length of the cable.
		double								Restitution;				// Holds the restitution (bounciness) of the cable.

		virtual uint32_t					AddContact					(ParticleContact *contact, uint32_t limit)					const;	// Fills the given contact structure with the contact needed to keep the cable from over-extending.
	};

	// Rods link a particle to an anchor point, generating a contact if they stray too far apart or too close.
	class ParticleRodConstraint : public ParticleConstraint {
	public:
		double								Length;	// Holds the length of the rod.
		virtual uint32_t					AddContact					(ParticleContact *contact, uint32_t limit)					const;	// Fills the given contact structure with the contact needed to keep the rod from extending or compressing.
	};
} // namespace cyclone

#endif // CYCLONE_CONTACTS_H