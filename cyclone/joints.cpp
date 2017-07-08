// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include "cyclone.h"

using namespace cyclone;

unsigned Joint::AddContact(Contact *contact, unsigned limit) const
{
    // Calculate the position of each connection point in world coordinates
    Vector3 a_pos_world = body[0]->getPointInWorldSpace(position[0]);
    Vector3 b_pos_world = body[1]->getPointInWorldSpace(position[1]);

    // Calculate the length of the joint
    Vector3 a_to_b = b_pos_world - a_pos_world;
    Vector3 normal = a_to_b;
    normal.normalise();
    real length = a_to_b.magnitude();

    // Check if it is violated
    if (real_abs(length) > Error)
    {
        contact->body[0] = body[0];
        contact->body[1] = body[1];
        contact->contactNormal = normal;
        contact->contactPoint = (a_pos_world + b_pos_world) * 0.5f;
        contact->penetration = length - Error;
        contact->friction = 1.0f;
        contact->restitution = 0;
        return 1;
    }

    return 0;
}

void Joint::set(RigidBody *a, const Vector3& a_pos,
                RigidBody *b, const Vector3& b_pos,
                real error)
{
    body[0] = a;
    body[1] = b;

    position[0] = a_pos;
    position[1] = b_pos;

    Joint::Error = error;
}