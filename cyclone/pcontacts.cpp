// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include "pcontacts.h"

using namespace cyclone;

// Contact implementation


double ParticleContact::CalculateSeparatingVelocity() const
{
    Vector3 relativeVelocity = Particle[0]->Velocity;
    if (Particle[1]) 
		relativeVelocity -= Particle[1]->Velocity;
    return relativeVelocity * ContactNormal;
}

void ParticleContact::ResolveVelocity(double duration) {
		
	double separatingVelocity = CalculateSeparatingVelocity();	// Find the velocity in the direction of the contact

	if (separatingVelocity > 0)	// Check if it needs to be resolved
		return;	// The contact is either separating, or stationary - there's no impulse required.
	
	double newSepVelocity = -separatingVelocity * Restitution;	// Calculate the new separating velocity
	
	// Check the velocity build-up due to acceleration only
	Vector3 accCausedVelocity = Particle[0]->Acceleration;
	if (Particle[1]) 
		accCausedVelocity -= Particle[1]->Acceleration;
	double accCausedSepVelocity = accCausedVelocity * ContactNormal * duration;
	
	if (accCausedSepVelocity < 0) {	// If we've got a closing velocity due to acceleration build-up, remove it from the new separating velocity
		newSepVelocity += Restitution * accCausedSepVelocity;
		if (newSepVelocity < 0) // Make sure we haven't removed more than was there to remove.
			newSepVelocity = 0;
	}

	double					deltaVelocity		= newSepVelocity - separatingVelocity;
	
	// We apply the change in velocity to each object in proportion to their inverse mass (i.e. those with lower inverse mass [higher actual mass] get less change in velocity)..
	double					totalInverseMass	= Particle[0]->InverseMass;
	if (Particle[1]) 
		totalInverseMass	+= Particle[1]->InverseMass;
	
	if (totalInverseMass <= 0)	// If all particles have infinite mass, then impulses have no effect
		return;
	
	
	double					impulse				= deltaVelocity / totalInverseMass;	// Calculate the impulse to apply
	Vector3					impulsePerIMass		= ContactNormal * impulse;			// Find the amount of impulse per unit of inverse mass
	
	// Apply impulses: they are applied in the direction of the contact, and are proportional to the inverse mass.
	Particle[0]->Velocity							= Particle[0]->Velocity + impulsePerIMass * Particle[0]->InverseMass;
	if (Particle[1])
		Particle[1]->Velocity						= Particle[1]->Velocity + impulsePerIMass * -Particle[1]->InverseMass;	// Particle 1 goes in the opposite direction
}

void ParticleContact::ResolveInterpenetration(double duration)
{
	if (Penetration <= 0)	// If we don't have any penetration, skip this step.
		return;

	// The movement of each object is based on their inverse mass, so total that.
	double totalInverseMass = Particle[0]->InverseMass;
	if (Particle[1]) 
		totalInverseMass += Particle[1]->InverseMass;

	if (totalInverseMass <= 0)	// If all particles have infinite mass, then we do nothing
		return;

	Vector3 movePerIMass = ContactNormal * (Penetration / totalInverseMass);	// Find the amount of penetration resolution per unit of inverse mass

	// Calculate the the movement amounts
	ParticleMovement[0] = movePerIMass * Particle[0]->InverseMass;
	if (Particle[1]) 
	    ParticleMovement[1] = movePerIMass * -Particle[1]->InverseMass;
	else
	    ParticleMovement[1].clear();

	// Apply the penetration resolution
	Particle[0]->Position = Particle[0]->Position + ParticleMovement[0];
	if (Particle[1]) 
	    Particle[1]->Position = Particle[1]->Position + ParticleMovement[1];
}

void ParticleContactResolver::ResolveContacts(ParticleContact *contactArray, uint32_t numContacts, double duration)
{
	IterationsUsed			= 0;
	while(IterationsUsed < Iterations) {	// Find the contact with the largest closing velocity;
		double					max						= REAL_MAX;
		uint32_t				maxIndex				= numContacts;
		for (uint32_t i = 0; i < numContacts; ++i) {
			double					sepVel					= contactArray[i].CalculateSeparatingVelocity();
			if (sepVel < max &&
				(sepVel < 0 || contactArray[i].Penetration > 0))
			{
				max					= sepVel;
				maxIndex			= i;
			}
		}
		
		if (maxIndex == numContacts)	// Do we have anything worth resolving?
			break;
		
		contactArray[maxIndex].Resolve(duration);	// Resolve this contact
		
		// Update the interpenetrations for all particles
		Vector3					* move					= contactArray[maxIndex].ParticleMovement;
		for (uint32_t i = 0; i < numContacts; ++i) {
				 if (contactArray[i].Particle[0] == contactArray[maxIndex].Particle[0]) contactArray[i].Penetration	-= move[0] * contactArray[i].ContactNormal;
			else if (contactArray[i].Particle[0] == contactArray[maxIndex].Particle[1])	contactArray[i].Penetration	-= move[1] * contactArray[i].ContactNormal;
			if (contactArray[i].Particle[1]) {
					 if (contactArray[i].Particle[1] == contactArray[maxIndex].Particle[0])	contactArray[i].Penetration	+= move[0] * contactArray[i].ContactNormal;
			    else if (contactArray[i].Particle[1] == contactArray[maxIndex].Particle[1]) contactArray[i].Penetration	+= move[1] * contactArray[i].ContactNormal;
			}
		}
		IterationsUsed++;
	}
}
