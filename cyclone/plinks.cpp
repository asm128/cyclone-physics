// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include "plinks.h"

using namespace cyclone;

real ParticleLink::currentLength() const
{
    Vector3 relativePos = particle[0]->Position -
                          particle[1]->Position;
    return relativePos.magnitude();
}

unsigned ParticleCable::AddContact(ParticleContact *contact, uint32_t limit) const
{
    // Find the length of the cable
    real length = currentLength();

    // Check if we're over-extended
    if (length < maxLength)
    {
        return 0;
    }

    // Otherwise return the contact
    contact->particle[0] = particle[0];
    contact->particle[1] = particle[1];

    // Calculate the normal
    Vector3 normal = particle[1]->Position - particle[0]->Position;
    normal.normalise();
    contact->contactNormal = normal;

    contact->penetration = length-maxLength;
    contact->restitution = restitution;

    return 1;
}

unsigned ParticleRod::AddContact(ParticleContact *contact,
                                  unsigned limit) const
{
    // Find the length of the rod
    real currentLen = currentLength();

    // Check if we're over-extended
    if (currentLen == length)
    {
        return 0;
    }

    // Otherwise return the contact
    contact->particle[0] = particle[0];
    contact->particle[1] = particle[1];

    // Calculate the normal
    Vector3 normal = particle[1]->Position - particle[0]->Position;
    normal.normalise();

    // The contact normal depends on whether we're extending or compressing
    if (currentLen > length) {
        contact->contactNormal = normal;
        contact->penetration = currentLen - length;
    } else {
        contact->contactNormal = normal * -1;
        contact->penetration = length - currentLen;
    }

    // Always use zero restitution (no bounciness)
    contact->restitution = 0;

    return 1;
}

real ParticleConstraint::currentLength() const
{
    Vector3 relativePos = particle->Position - anchor;
    return relativePos.magnitude();
}

unsigned ParticleCableConstraint::AddContact(ParticleContact *contact,
                                   unsigned limit) const
{
    // Find the length of the cable
    real length = currentLength();

    // Check if we're over-extended
    if (length < maxLength)
    {
        return 0;
    }

    // Otherwise return the contact
    contact->particle[0] = particle;
    contact->particle[1] = 0;

    // Calculate the normal
    Vector3 normal = anchor - particle->Position;
    normal.normalise();
    contact->contactNormal = normal;

    contact->penetration = length-maxLength;
    contact->restitution = restitution;

    return 1;
}

unsigned ParticleRodConstraint::AddContact(ParticleContact *contact,
                                 unsigned limit) const
{
    // Find the length of the rod
    real currentLen = currentLength();

    // Check if we're over-extended
    if (currentLen == length)
    {
        return 0;
    }

    // Otherwise return the contact
    contact->particle[0] = particle;
    contact->particle[1] = 0;

    // Calculate the normal
    Vector3 normal = anchor - particle->Position;
    normal.normalise();

    // The contact normal depends on whether we're extending or compressing
    if (currentLen > length) {
        contact->contactNormal = normal;
        contact->penetration = currentLen - length;
    } else {
        contact->contactNormal = normal * -1;
        contact->penetration = length - currentLen;
    }

    // Always use zero restitution (no bounciness)
    contact->restitution = 0;

    return 1;
}