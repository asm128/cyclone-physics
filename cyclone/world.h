// This file contains the definitions for a structure to hold any number of rigid bodies, and to manage their simulation.
// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include "body.h"
#include "contacts.h"

#ifndef CYCLONE_WORLD_H
#define CYCLONE_WORLD_H

namespace cyclone {
	// The world represents an independent simulation of physics. It keeps track of a set of rigid bodies, and provides the means to update them all.
	// If you don't give a number of iterations, then four times the number of detected contacts will be used for each frame.
	class World {
		bool	calculateIterations;	// True if the world should calculate the number of iterations to give the contact resolver at each frame.
		// Holds a single rigid body in a linked list of bodies.
		struct	BodyRegistration {
			RigidBody					* body						= 0;
			BodyRegistration			* next						= 0;
		};

		
		BodyRegistration			* firstBody					= 0;	// Holds the head of the list of registered bodies.
		ContactResolver				resolver					;					// Holds the resolver for sets of contacts.

		// Holds one contact generators in a linked list.
		struct ContactGenRegistration {
			ContactGenerator			* gen						= 0;
			ContactGenRegistration		* next						= 0;
		};

		ContactGenRegistration		* firstContactGen			= 0;	// Holds the head of the list of contact generators.
		Contact						* contacts					= 0;	// Holds an array of contacts, for filling by the contact generators.
		uint32_t					maxContacts;	// Holds the maximum number of contacts allowed (i.e. the size of the contacts array).

	public:
		// Creates a new simulator that can handle up to the given number of contacts per frame. You can also optionally give a number of contact-resolution iterations to use. 
									World						(uint32_t maxContacts, uint32_t iterations = 0);
									~World						();

		uint32_t					GenerateContacts			();	// Calls each of the registered contact generators to report their contacts. Returns the number of generated contacts.
		void						runPhysics					(double duration);	// Processes all the physics for the world.
		void						startFrame					();	// Initialises the world for a simulation frame. This clears the force and torque accumulators for bodies in the world. After calling this, the bodies can have their forces and torques for this frame added.
	};
} // namespace cyclone

#endif // CYCLONE_PWORLD_H
