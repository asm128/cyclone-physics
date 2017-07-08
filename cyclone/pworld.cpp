/*
 * Implementation file for random number generation.
 *
 * Part of the Cyclone physics system.
 *
 * Copyright (c) Icosagon 2003. All Rights Reserved.
 *
 * This software is distributed under licence. Use of this software
 * implies agreement with all terms and conditions of the accompanying
 * software licence.
 */
#include "pworld.h"

#include <cstddef>

using namespace cyclone;

ParticleWorld::ParticleWorld(unsigned maxContacts, unsigned iterations)
	: Resolver		(iterations	)
	, MaxContacts	(maxContacts)
{
	Contacts				= new ParticleContact[maxContacts];
	CalculateIterations		= (iterations == 0);
}

ParticleWorld::~ParticleWorld()
{
	if(Contacts)
		delete[] Contacts;
}

void ParticleWorld::startFrame() {
    for (TParticles::iterator p = Particles.begin(); p != Particles.end(); ++p)
        (*p)->clearAccumulator();	// Remove all forces from the accumulator
}

unsigned ParticleWorld::GenerateContacts()
{
    uint32_t			limit			= MaxContacts;
    ParticleContact		* nextContact	= Contacts;

    for (TContactGenerators::iterator g = ContactGenerators.begin(); g != ContactGenerators.end(); ++g) {
        uint32_t used =(*g)->AddContact(nextContact, limit);
        limit -= used;
        nextContact += used;

        // We've run out of contacts to fill. This means we're missing
        // contacts.
        if (limit <= 0) 
			break;
    }

    // Return the number of contacts used.
    return MaxContacts - limit;
}

void ParticleWorld::integrate(real duration) {
	for (TParticles::iterator p = Particles.begin(); p != Particles.end(); ++p)
		(*p)->integrate(duration);		// Remove all forces from the accumulator
}

void ParticleWorld::runPhysics(real duration)
{
    // First apply the force generators
    Registry.updateForces(duration);

    // Then integrate the objects
    integrate(duration);

    // Generate contacts
    uint32_t usedContacts = GenerateContacts();

    // And process them
    if (usedContacts)
    {
        if (CalculateIterations) 
			Resolver.setIterations(usedContacts * 2);
        Resolver.resolveContacts(Contacts, usedContacts, duration);
    }
}

ParticleWorld::TParticles			&	ParticleWorld::getParticles			() { return Particles;			}
ParticleWorld::TContactGenerators	&	ParticleWorld::getContactGenerators	() { return ContactGenerators;	}
ParticleForceRegistry				&	ParticleWorld::getForceRegistry		() { return Registry;			}

void GroundContacts::Init(cyclone::ParticleWorld::TParticles *particles)
{
    Particles = particles;
}

unsigned GroundContacts::AddContact(cyclone::ParticleContact *contact, uint32_t limit) const
{
    unsigned count = 0;
    for (cyclone::ParticleWorld::TParticles::iterator p = Particles->begin(); p != Particles->end(); ++p)
    {
        cyclone::real y = (*p)->getPosition().y;
        if (y < 0.0f)
        {
            contact->contactNormal = cyclone::Vector3::UP;
            contact->particle[0] = *p;
            contact->particle[1] = NULL;
            contact->penetration = -y;
            contact->restitution = 0.2f;
            contact++;
            count++;
        }

        if (count >= limit) return count;
    }
    return count;
}