// Copyright (c) Icosagon 2003. All Rights Reserved.
#include "particle.h"

#include <assert.h>

using namespace cyclone;
void Particle::integrate(real duration) {
	if (InverseMass <= 0.0f)	// We don't integrate things with zero mass.
		return;

	Position.addScaledVector(Velocity, duration);	// Update linear position.
	
	// Work out the acceleration from the force
	Vector3						resultingAcc				= Acceleration;
	resultingAcc	.addScaledVector(ForceAccum, InverseMass);
	Velocity		.addScaledVector(resultingAcc, duration);	// Update linear velocity from the acceleration.
	Velocity				*= real_pow(Damping, duration);		// Impose drag.
	
	clearAccumulator();	// Clear the forces.
}



void Particle::setMass(const real mass) {
	InverseMass = ((real)1.0) / mass;
}

real Particle::getMass() const {
	return (InverseMass == 0) ? REAL_MAX : ((real)1.0) / InverseMass;
}

bool Particle::hasFiniteMass() const {
	return InverseMass >= 0.0f;
}

void Particle::getPosition(Vector3 *position) const {
	*position = Position;
}


void Particle::getVelocity(Vector3 *velocity) const
{
    *velocity = Velocity;
}

void Particle::getAcceleration(Vector3 *acceleration) const
{
    *acceleration = Acceleration;
}

void Particle::clearAccumulator()
{
    ForceAccum.clear();
}

void Particle::addForce(const Vector3 &force)
{
    ForceAccum += force;
}
