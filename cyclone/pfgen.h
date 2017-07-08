// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices, obsolete tools and OOP vices.
#ifndef CYCLONE_PFGEN_H
#define CYCLONE_PFGEN_H

#include "core.h"
#include "particle.h"
#include <vector>

namespace cyclone {
	// A force generator can be asked to add a force to one or more particles.
	class ParticleForceGenerator {
	public:
		virtual	void									updateForce						(Particle *particle, real duration)									= 0;	// Overload this in implementations of the interface to calculate and update the force applied to the given particle.
	};
	
	// A force generator that applies a gravitational force. One instance can be used for multiple particles.
	class ParticleGravity : public ParticleForceGenerator {
				Vector3									gravity;	// Holds the acceleration due to gravity.
	
	public:
														ParticleGravity					(const Vector3 &gravity);				// Creates the generator with the given acceleration. 
		virtual	void									updateForce						(Particle *particle, real duration);	// Applies the gravitational force to the given particle. 
	};
	
	// A force generator that applies a drag force. One instance can be used for multiple particles.
	class ParticleDrag : public ParticleForceGenerator {
				real									k1;	// Holds the velocity drag coeffificent.
				real									k2;	// Holds the velocity squared drag coeffificent.
	
	public:
														ParticleDrag					(real k1, real k2);						// Creates the generator with the given coefficients. 
		virtual	void									updateForce						(Particle *particle, real duration);	// Applies the drag force to the given particle. 
	};
	
	// A force generator that applies a Spring force, where one end is attached to a fixed point in space.
	class ParticleAnchoredSpring : public ParticleForceGenerator {
	protected:
				Vector3									* anchor;		// The location of the anchored end of the spring. 
				real									springConstant;	// Holds the sprint constant. 
				real									restLength;		// Holds the rest length of the spring. 
	
	public:
														ParticleAnchoredSpring			()																	= default;
														ParticleAnchoredSpring			(Vector3 *anchor, real springConstant, real restLength);	
	
				const Vector3*							getAnchor						()															const	{ return anchor; }	
				void									init							(Vector3 *anchor, real springConstant, real restLength);	// Set the spring's properties. 
		virtual	void									updateForce						(Particle *particle, real duration);						// Applies the spring force to the given particle.
	};
	
	// A force generator that applies a bungee force, where one end is attached to a fixed point in space.
	class ParticleAnchoredBungee : public ParticleAnchoredSpring {
	public:
		virtual	void									updateForce						(Particle *particle, real duration);	// Applies the spring force to the given particle.
	};
	
	// A force generator that fakes a stiff spring force, and where one end is attached to a fixed point in space.
	class ParticleFakeSpring : public ParticleForceGenerator {
				Vector3									* anchor						= 0;	// The location of the anchored end of the spring. 
				double									springConstant					= 0;	// Holds the sprint constant. 
				double									damping							= 0;	// Holds the damping on the oscillation of the spring.
	
	public:
														ParticleFakeSpring				(Vector3 *anchor, real springConstant, real damping);	// Creates a new spring with the given parameters.
	
		virtual void									updateForce						(Particle *particle, real duration);	// Applies the spring force to the given particle. 
	};
	
	// A force generator that applies a Spring force.
	class ParticleSpring : public ParticleForceGenerator {
				Particle								* other							= 0;	// The particle at the other end of the spring.
				double									springConstant					= 0;	// Holds the sprint constant.
				double									restLength						= 0;	// Holds the rest length of the spring.
	
	public:
														ParticleSpring					(Particle *other, real springConstant, real restLength);	// Creates a new spring with the given parameters. 
		virtual void									updateForce						(Particle *particle, real duration);						// Applies the spring force to the given particle. 
	};
	
	// A force generator that applies a spring force only when extended.
	class ParticleBungee : public ParticleForceGenerator {
				Particle								* other							= 0;	// The particle at the other end of the spring.
				double									springConstant					= 0;	// Holds the sprint constant.
				double									restLength						= 0;	// Holds the length of the bungee at the point it begins to generator a force.
	
	public:
														ParticleBungee					(Particle *other, real springConstant, real restLength); 
		virtual void									updateForce						(Particle *particle, real duration);	// Applies the spring force to the given particle.
	};
	
	// A force generator that applies a buoyancy force for a plane of liquid parrallel to XZ plane.
	class ParticleBuoyancy : public ParticleForceGenerator {
				double									maxDepth						= 0;	// The maximum submersion depth of the object before it generates its maximum boyancy force.
				double									volume							= 0;	// The volume of the object.
				double									waterHeight						= 0;	// The height of the water plane above y=0. The plane will be parrallel to the XZ plane.
				double									liquidDensity					= 0;	// The density of the liquid. Pure water has a density of 1000kg per cubic meter.
	
	public:
		// Creates a new buoyancy force with the given parameters.
														ParticleBuoyancy				(real maxDepth, real volume, real waterHeight, real liquidDensity = 1000.0f);
	
		virtual	void									updateForce						(Particle *particle, real duration);	// Applies the buoyancy force to the given particle.
	};
	
	// Holds all the force generators and the particles they apply to.
	class ParticleForceRegistry {
	protected:
		// Keeps track of one force generator and the particle it applies to.
		struct ParticleForceRegistration {
					Particle								* particle;
					ParticleForceGenerator					* fg;
		};
		
		typedef	std::vector<ParticleForceRegistration>	TRegistry;	
				TRegistry								registrations;										// Holds the list of registrations.
	
	public:
				void									add								(Particle* particle, ParticleForceGenerator *fg);	// Registers the given force generator to apply to the given particle.
				void									remove							(Particle* particle, ParticleForceGenerator *fg);	// Removes the given registered pair from the registry. If the pair is not registered, this method will have no effect.
				void									clear							();													// Clears all registrations from the registry. This will not delete the particles or the force generators themselves, just the records of their connection.
				void									updateForces					(real duration);									// Calls all the force generators to update the forces of their corresponding particles.
	};
}

#endif // CYCLONE_PFGEN_H