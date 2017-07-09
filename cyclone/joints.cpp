// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include "cyclone.h"

using namespace cyclone;

uint32_t Joint::AddContact(Contact *contact, uint32_t limit) const
{
    // Calculate the position of each connection point in world coordinates
    Vector3 a_pos_world = Body[0]->getPointInWorldSpace(Position[0]);
    Vector3 b_pos_world = Body[1]->getPointInWorldSpace(Position[1]);

    // Calculate the length of the joint
    Vector3 a_to_b = b_pos_world - a_pos_world;
    Vector3 normal = a_to_b;
    normal.normalise();
    double length = a_to_b.magnitude();

    // Check if it is violated
    if (real_abs(length) > Error)
    {
        contact->Body[0]		= Body[0];
        contact->Body[1]		= Body[1];
        contact->ContactNormal	= normal;
        contact->ContactPoint	= (a_pos_world + b_pos_world) * 0.5f;
        contact->Penetration	= length - Error;
        contact->Friction		= 1.0f;
        contact->Restitution	= 0;
        return 1;
    }

    return 0;
}

void Joint::Set(RigidBody *a, const Vector3& a_pos,
                RigidBody *b, const Vector3& b_pos,
                double error)
{
    Body[0] = a;
    Body[1] = b;

    Position[0] = a_pos;
    Position[1] = b_pos;

    Joint::Error = error;
}