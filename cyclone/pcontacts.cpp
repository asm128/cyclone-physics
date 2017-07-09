// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include "pcontacts.h"

using namespace cyclone;

// Contact implementation

void ParticleContact::resolve(real duration)
{
    resolveVelocity			(duration);
    resolveInterpenetration	(duration);
}

real ParticleContact::calculateSeparatingVelocity() const
{
    Vector3 relativeVelocity = particle[0]->Velocity;
    if (particle[1]) 
		relativeVelocity -= particle[1]->Velocity;
    return relativeVelocity * contactNormal;
}

void ParticleContact::resolveVelocity(real duration) {
		
	real separatingVelocity = calculateSeparatingVelocity();	// Find the velocity in the direction of the contact

	if (separatingVelocity > 0)	// Check if it needs to be resolved
		return;	// The contact is either separating, or stationary - there's no impulse required.
	
	real newSepVelocity = -separatingVelocity * restitution;	// Calculate the new separating velocity
	
	// Check the velocity build-up due to acceleration only
	Vector3 accCausedVelocity = particle[0]->Acceleration;
	if (particle[1]) 
		accCausedVelocity -= particle[1]->Acceleration;
	real accCausedSepVelocity = accCausedVelocity * contactNormal * duration;
	
	if (accCausedSepVelocity < 0) {	// If we've got a closing velocity due to acceleration build-up, remove it from the new separating velocity
		newSepVelocity += restitution * accCausedSepVelocity;
		if (newSepVelocity < 0) // Make sure we haven't removed more than was there to remove.
			newSepVelocity = 0;
	}

	real					deltaVelocity		= newSepVelocity - separatingVelocity;
	
	// We apply the change in velocity to each object in proportion to their inverse mass (i.e. those with lower inverse mass [higher actual mass] get less change in velocity)..
	real					totalInverseMass	= particle[0]->InverseMass;
	if (particle[1]) 
		totalInverseMass	+= particle[1]->InverseMass;
	
	if (totalInverseMass <= 0)	// If all particles have infinite mass, then impulses have no effect
		return;
	
	
	real					impulse				= deltaVelocity / totalInverseMass;	// Calculate the impulse to apply
	Vector3					impulsePerIMass		= contactNormal * impulse;			// Find the amount of impulse per unit of inverse mass
	
	// Apply impulses: they are applied in the direction of the contact, and are proportional to the inverse mass.
	particle[0]->Velocity							= particle[0]->Velocity + impulsePerIMass * particle[0]->InverseMass;
	if (particle[1])
		particle[1]->Velocity						= particle[1]->Velocity + impulsePerIMass * -particle[1]->InverseMass;	// Particle 1 goes in the opposite direction
}

void ParticleContact::resolveInterpenetration(real duration)
{
	if (penetration <= 0)	// If we don't have any penetration, skip this step.
		return;

    // The movement of each object is based on their inverse mass, so total that.
    real totalInverseMass = particle[0]->InverseMass;
    if (particle[1]) 
		totalInverseMass += particle[1]->InverseMass;

    if (totalInverseMass <= 0)	// If all particles have infinite mass, then we do nothing
		return;

    
    Vector3 movePerIMass = contactNormal * (penetration / totalInverseMass);	// Find the amount of penetration resolution per unit of inverse mass

    // Calculate the the movement amounts
    particleMovement[0] = movePerIMass * particle[0]->InverseMass;
    if (particle[1]) 
        particleMovement[1] = movePerIMass * -particle[1]->InverseMass;
    else
        particleMovement[1].clear();

    // Apply the penetration resolution
    particle[0]->Position = particle[0]->Position + particleMovement[0];
    if (particle[1]) 
        particle[1]->Position = particle[1]->Position + particleMovement[1];
}

void ParticleContactResolver::resolveContacts(ParticleContact *contactArray, uint32_t numContacts, double duration)
{
	IterationsUsed			= 0;
	while(IterationsUsed < Iterations) {	// Find the contact with the largest closing velocity;
		real					max = REAL_MAX;
		unsigned				maxIndex = numContacts;
		for (uint32_t i = 0; i < numContacts; ++i) {
			real sepVel = contactArray[i].calculateSeparatingVelocity();
			if (sepVel < max &&
				(sepVel < 0 || contactArray[i].penetration > 0))
			{
				max = sepVel;
				maxIndex = i;
			}
		}
		
		if (maxIndex == numContacts)	// Do we have anything worth resolving?
			break;
		
		contactArray[maxIndex].resolve(duration);	// Resolve this contact
		
		// Update the interpenetrations for all particles
		Vector3 *move = contactArray[maxIndex].particleMovement;
		for (uint32_t i = 0; i < numContacts; ++i) {
			if (contactArray[i].particle[0] == contactArray[maxIndex].particle[0])
			    contactArray[i].penetration		-= move[0] * contactArray[i].contactNormal;
			else if (contactArray[i].particle[0] == contactArray[maxIndex].particle[1])
			    contactArray[i].penetration		-= move[1] * contactArray[i].contactNormal;
			if (contactArray[i].particle[1]) {
			    if (contactArray[i].particle[1] == contactArray[maxIndex].particle[0])
			        contactArray[i].penetration		+= move[0] * contactArray[i].contactNormal;
			    else if (contactArray[i].particle[1] == contactArray[maxIndex].particle[1])
			        contactArray[i].penetration		+= move[1] * contactArray[i].contactNormal;
			}
		}
		IterationsUsed++;
	}
}
