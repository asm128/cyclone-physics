// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include "particle.h"

#ifndef CYCLONE_PCONTACTS_H
#define CYCLONE_PCONTACTS_H

namespace cyclone {
	class ParticleContactResolver;	
	
	// A Contact represents two objects in contact (in this case ParticleContact representing two Particles). 
	// Resolving a contact removes their interpenetration, and applies sufficient impulse to keep them apart. Colliding bodies may also rebound.
	// The contact has no callable functions, it just holds the contact details. To resolve a set of contacts, use the particle contact resolver class.
	class ParticleContact {
		friend	class						ParticleContactResolver;	// The contact resolver object needs access into the contacts to set and effect the contact.
	public:
				Particle*					Particle			[2]				= {};	// Holds the particles that are involved in the contact. The second of these can be NULL, for contacts with the scenery.
				double						Restitution							= {};	// Holds the normal restitution coefficient at the contact.
				Vector3						ContactNormal						= {};	// Holds the direction of the contact in world coordinates.
				double						Penetration							= {};	// Holds the depth of penetration at the contact.

				
				Vector3						ParticleMovement	[2]				= {};	// Holds the amount each particle is moved by during interpenetration resolution.

	protected:
		inline	void						Resolve								(double duration)																{
			ResolveVelocity			(duration);
			ResolveInterpenetration	(duration);
		}															// Resolves this contact, for both velocity and interpenetration.
				double						CalculateSeparatingVelocity			()																		const;	// Calculates the separating velocity at this contact.
	private:
				void						ResolveVelocity						(double duration);			// Handles the impulse calculations for this collision.
				void						ResolveInterpenetration				(double duration);			// Handles the interpenetration resolution for this contact.
	};

	// The contact resolution routine for particle contacts. One resolver instance can be shared for the whole simulation.
	class ParticleContactResolver {
	protected:
				uint32_t					Iterations							= 0;	// Holds the number of iterations allowed.
				uint32_t					IterationsUsed						= 0;	// This is a performance tracking value - we keep a record of the actual number of iterations used.
	public:
		inline	constexpr					ParticleContactResolver				(uint32_t iterations)															: Iterations(iterations)	{}
											
				void						SetIterations						(uint32_t iterations)															{ Iterations = iterations;	}
		// Resolves a set of particle contacTs for both penetration and velocity.
		// Contacts that cannot interact witH each other should be passed to separate calls to resolveContacts, as the resolution algorithm takes much longer for lots of contacts than it does for the same number of contacts in small sets.
		// contactArray		: Pointer to an Array of particle contact objects.
		// numContacts		: The number of Contacts in the array to resolve.
		// numIterations	: The number of Iterations through the resolution algorithm. This should be at least the number of contacts (otherwise some constraints will not be resolved - although sometimes this is not noticable). 
		//					If the iterationS are not needed they will not be used, so adding more iterations may not make any difference. But in some cases you would need millions of iterations. 
		//					Think about the Number of iterations as a bound: if you specify a large number, sometimes the algorithm WILL use it, and you may drop frames.
		//
		// duration			: The duration oF the previous integration step. This is used to compensate for forces applied.
				void						ResolveContacts						(ParticleContact *contactArray, uint32_t numContacts, double duration);
	};

	// This is the basic polymorphic interface for contact generators applying to particles.
	struct ParticleContactGenerator {
		virtual	uint32_t					AddContact							(ParticleContact *contact, uint32_t limit)								const	= 0;
	};
} // namespace cyclone

#endif // CYCLONE_CONTACTS_H
