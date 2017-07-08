// Copyright (c) Icosagon 2003. All Rights Reserved.

#ifndef CYCLONE_PARTICLE_H
#define CYCLONE_PARTICLE_H

#include "core.h"

namespace cyclone {

	// A particle is the simplest object that can be simulated in the physics system.
	//
	// It has position data (no orientation data), along with velocity. It can be integrated forward through time, and have linear forces, and impulses applied to it. The particle manages its state and allows access through a set of methods.
	struct Particle {
		double				InverseMass;
		double				Damping;
		Vector3				Position;
		Vector3				Velocity;
		Vector3				ForceAccum;
		Vector3				Acceleration;

		void				integrate			(real duration);
		void				setMass				(const real mass);
		real				getMass				()						const;
		bool				hasFiniteMass		()						const;
		void				getPosition			(Vector3 *position)		const;
		void				getVelocity			(Vector3 *velocity)		const;
		void				getAcceleration		(Vector3 *acceleration)	const;
		void				clearAccumulator	();
		void				addForce			(const Vector3 &force);
	};
}

#endif // CYCLONE_BODY_H