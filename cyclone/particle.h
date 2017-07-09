// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include "core.h"

#ifndef CYCLONE_PARTICLE_H
#define CYCLONE_PARTICLE_H

namespace cyclone {

	// A particle is the simplest object that can be simulated in the physics system.
	// It has position data (no orientation data), along with velocity. It can be integrated forward through time, and have linear forces, and impulses applied to it. The particle manages its state and allows access through a set of methods.
	struct Particle {
		double					InverseMass						= 0;
		double					Damping							= 0;
		Vector3					Position						= {};
		Vector3					Velocity						= {};
		Vector3					AccumulatedForce				= {};
		Vector3					Acceleration					= {};

		void					setMass							(const double mass)					{ InverseMass = ((double)1.0) / mass;									}
		double					getMass							()							const	{ return (InverseMass == 0) ? REAL_MAX : ((double)1.0) / InverseMass;	}
		bool					hasFiniteMass					()							const	{ return InverseMass >= 0.0f;											}
		void					clearAccumulator				()									{ AccumulatedForce.clear();												}
		void					addForce						(const Vector3 &force)				{ AccumulatedForce += force;											}
		void					integrate						(double duration)					{
			if (InverseMass <= 0.0f)	// We don't integrate things with infinite mass.
				return;
		
			Position.addScaledVector(Velocity, duration);	// Update linear position.
			
			// Work out the acceleration from the force
			Vector3						resultingAcc				= Acceleration;
			resultingAcc	.addScaledVector(AccumulatedForce, InverseMass);
			Velocity		.addScaledVector(resultingAcc, duration);	// Update linear velocity from the acceleration.
			Velocity				*= real_pow(Damping, duration);		// Impose drag.
			
			clearAccumulator();	// Clear the forces.
		}
	};
}

#endif // CYCLONE_BODY_H