// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices, obsolete tools and OOP vices.
#include "core.h"
#include "particle.h"

#include <vector>

#ifndef CYCLONE_PFGEN_H
#define CYCLONE_PFGEN_H

namespace cyclone {
	// A force generator can be asked to add a force to one or more particles.
	class ParticleForceGenerator {
	public:
		virtual	void									UpdateForce						(Particle *particle, double duration)													= 0;	// Overload this in implementations of the interface to calculate and update the force applied to the given particle.
	};
	
	
	// Holds all the force generators and the particles they apply to.
	struct ParticleForceRegistry {
		// Keeps track of one force generator and the particle it applies to.
		struct ParticleForceRegistration {
					Particle								* Particle;
					ParticleForceGenerator					* ForceGenerator;
		};
		
		typedef	std::vector<ParticleForceRegistration>	TRegistry;	
				TRegistry								Registrations;										// Holds the list of registrations.
	
				void									UpdateForces					(double duration);									// Calls all the force generators to update the forces of their corresponding particles.
	};

	// A force generator that applies a gravitational force. One instance can be used for multiple particles.
	class ParticleGravity : public ParticleForceGenerator {
				Vector3									Gravity;	// Holds the acceleration due to gravity.
	
	public:
														ParticleGravity					(const Vector3& gravity)																: Gravity(gravity)																				{}

		virtual	void									UpdateForce						(Particle *particle, double duration);	// Applies the gravitational force to the given particle. 
	};
	
	// A force generator that applies a drag force. One instance can be used for multiple particles.
	class ParticleDrag : public ParticleForceGenerator {
				double									k1;	// Holds the velocity drag coeffificent.
				double									k2;	// Holds the velocity squared drag coeffificent.
	
	public:
														ParticleDrag					(double k1, double k2)																	: k1(k1), k2(k2)																				{}

		virtual	void									UpdateForce						(Particle *particle, double duration);	// Applies the drag force to the given particle. 
	};
	
	// A force generator that applies a Spring force, where one end is attached to a fixed point in space.
	class ParticleAnchoredSpring : public ParticleForceGenerator {
	protected:
				Vector3									* Anchor						= 0;	// The location of the anchored end of the spring. 
				double									SpringConstant					= 0;	// Holds the sprint constant. 
				double									RestLength						= 0;	// Holds the rest length of the spring. 
	
	public:
														ParticleAnchoredSpring			()																						= default;
														ParticleAnchoredSpring			(Vector3 *anchor, double springConstant, double restLength)								: Anchor(anchor), SpringConstant(springConstant), RestLength(restLength)						{}
	
				const Vector3*							GetAnchor						()																				const	{ return Anchor; }	
				void									Init							(Vector3 *anchor, double springConstant, double restLength);	// Set the spring's properties. 

		virtual	void									UpdateForce						(Particle *particle, double duration);							// Applies the spring force to the given particle.
	};
	
	// A force generator that applies a bungee force, where one end is attached to a fixed point in space.
	class ParticleAnchoredBungee : public ParticleAnchoredSpring {
	public:
		virtual	void									UpdateForce						(Particle *particle, double duration);	// Applies the spring force to the given particle.
	};
	
	// A force generator that fakes a stiff spring force, and where one end is attached to a fixed point in space.
	class ParticleFakeSpring : public ParticleForceGenerator {
				Vector3									* Anchor						= 0;	// The location of the anchored end of the spring. 
				double									SpringConstant					= 0;	// Holds the sprint constant. 
				double									Damping							= 0;	// Holds the damping on the oscillation of the spring.
	
	public:
														ParticleFakeSpring				(Vector3 *anchor, double springConstant, double damping)								: Anchor(anchor), SpringConstant(springConstant), Damping(damping)								{}
	
		virtual void									UpdateForce						(Particle *particle, double duration);	// Applies the spring force to the given particle. 
	};
	
	// A force generator that applies a Spring force.
	class ParticleSpring : public ParticleForceGenerator {
				Particle								* Other							= 0;	// The particle at the other end of the spring.
				double									SpringConstant					= 0;	// Holds the sprint constant.
				double									RestLength						= 0;	// Holds the rest length of the spring.
	
	public:
														ParticleSpring					(Particle *other, double springConstant, double restLength)								: Other(other), SpringConstant(springConstant), RestLength(restLength)							{}

		virtual void									UpdateForce						(Particle *particle, double duration);						// Applies the spring force to the given particle. 
	};
	
	// A force generator that applies a spring force only when extended.
	class ParticleBungee : public ParticleForceGenerator {
				Particle								* Other							= 0;	// The particle at the other end of the spring.
				double									SpringConstant					= 0;	// Holds the sprint constant.
				double									RestLength						= 0;	// Holds the length of the bungee at the point it begins to generator a force.
	
	public:
														ParticleBungee					(Particle *other, double springConstant, double restLength)								: Other(other), SpringConstant(springConstant), RestLength(restLength)							{}

		virtual void									UpdateForce						(Particle *particle, double duration);	// Applies the spring force to the given particle.
	};

	// A force generator that applies a buoyancy force for a plane of liquid parrallel to XZ plane.
	class ParticleBuoyancy : public ParticleForceGenerator {
				double									MaxDepth						= 0;	// The maximum submersion depth of the object before it generates its maximum boyancy force.
				double									Volume							= 0;	// The volume of the object.
				double									WaterHeight						= 0;	// The height of the water plane above y=0. The plane will be parrallel to the XZ plane.
				double									LiquidDensity					= 0;	// The density of the liquid. Pure water has a density of 1000kg per cubic meter.
	
	public:
														ParticleBuoyancy				(double maxDepth, double volume, double waterHeight, double liquidDensity = 1000.0f)	: MaxDepth(maxDepth), Volume(volume), WaterHeight(waterHeight), LiquidDensity(liquidDensity)	{}
	
		virtual	void									UpdateForce						(Particle *particle, double duration);	// Applies the buoyancy force to the given particle.
	};
}

#endif // CYCLONE_PFGEN_H