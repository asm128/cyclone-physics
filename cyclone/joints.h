// This file contains the definitions for joints that link together different rigid bodies.
// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include "contacts.h"

#ifndef CYCLONE_JOINTS_H
#define CYCLONE_JOINTS_H

namespace cyclone {

    // Joints link together two rigid bodies and make sure they do not separate. In a general phyiscs engine there may be many different types of joint, that reduce the number of relative degrees of freedom between two objects. 
	// This joint is a common position joint: each object has a location (given in body-coordinates) that will be kept at the same point in the simulation.
    class Joint : public ContactGenerator
    {
    public:
        RigidBody	* body		[2]		= {};	// Holds the two rigid bodies that are connected by this joint.
        Vector3		position	[2]		= {};	// Holds the relative location of the connection for each body, given in local coordinates.

        // Holds the maximum displacement at the joint before the joint is considered to be violated. This is normally a small, epsilon value. 
		// It can be larger, however, in which case the joint will behave as if an inelastic cable joined the bodies at their joint locations.
        double		Error;

        // Configures the joint in one go.
        void		set
			( RigidBody *a, const Vector3& a_pos
            , RigidBody *b, const Vector3& b_pos
            , double error
            );

        
        uint32_t	AddContact			(Contact *contact, uint32_t limit) const;	// Generates the contacts required to restore the joint if it has been violated.
    };

} // namespace cyclone

#endif // CYCLONE_JOINTS_H