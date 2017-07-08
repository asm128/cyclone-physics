// Copyright (c) Icosagon 2003. All Rights Reserved.
#ifndef CYCLONE_PWORLD_H
#define CYCLONE_PWORLD_H

#include "pfgen.h"
#include "plinks.h"

namespace cyclone {

	// Keeps track of a set of particles, and provides the means to update them all.
	struct ParticleWorld {
		typedef	::std::vector<Particle*>					TParticles;
		typedef	::std::vector<ParticleContactGenerator*>	TContactGenerators;
		
		TParticles											Particles				= {};				// Holds the particles
		bool												CalculateIterations		= false;			// True if the world should calculate the number of iterations to give the contact resolver at each frame.
		ParticleForceRegistry								Registry				= {};				// Holds the force generators for the particles in this world.
		ParticleContactResolver								Resolver				= 0;				// Holds the resolver for contacts.
		TContactGenerators									ContactGenerators		= {};				// Contact generators.
		ParticleContact										* Contacts				= 0;				// Holds the list of contacts.
		uint32_t											MaxContacts				= 0;				// Holds the maximum number of contacts allowed (i.e. the size of the contacts array).
		
															ParticleWorld			(uint32_t maxContacts, uint32_t iterations = 0);	// Creates a new particle simulator that can handle up to the given number of contacts per frame. You can also optionally give a number of contact-resolution iterations to use. If you don't give a number of iterations, then twice the number of contacts will be used.
															~ParticleWorld			();	

		uint32_t											GenerateContacts		();	// Calls each of the registered contact generators to report their contacts. Returns the number of generated contacts.
		void												integrate				(double duration);	// Integrates all the particles in this world forward in time by the given duration.
		void												runPhysics				(real duration);	// Processes all the physics for the particle world.
		void												startFrame				();	// Initializes the world for a simulation frame. This clears the force accumulators for particles in the world. After calling this, the particles can have their forces for this frame added.
		TParticles&											getParticles			();	//  Returns the list of particles.
		TContactGenerators&									getContactGenerators	();	// Returns the list of contact generators.
		
		ParticleForceRegistry&								getForceRegistry		();	// Returns the force registry.
	};

	// A contact generator that takes an STL vector of particle pointers and collides them against the ground.
	class GroundContacts : public cyclone::ParticleContactGenerator {
		cyclone::ParticleWorld::TParticles					* Particles				= 0;
	
	public:
		void												Init					(cyclone::ParticleWorld::TParticles *particles);
		virtual uint32_t									AddContact				(cyclone::ParticleContact *contact, unsigned limit)		const;
	};
} // namespace cyclone

#endif // CYCLONE_PWORLD_H