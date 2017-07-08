// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#ifndef CYCLONE_COLLISION_FINE_H
#define CYCLONE_COLLISION_FINE_H

#include "contacts.h"

namespace cyclone {
	// Forward declarations of primitive friends
	struct IntersectionTests;
	struct CollisionDetector;
	
	// Represents a primitive to detect collisions against.
	struct CollisionPrimitive {
		RigidBody					* Body								= 0;		// The rigid body that is represented by this primitive.
		Matrix4						Offset								= {};		// The offset of this primitive from the given rigid body.
		Matrix4						Transform;	// The resultant transform of the primitive. This is calculated by combining the offset of the primitive with the transform of the rigid body.

		void						CalculateInternals					()									noexcept	{ Transform = Body->getTransform() * Offset;	}	// Calculates the internals for the primitive.
		inline Vector3				GetAxis								(uint32_t index)			const	noexcept	{ return Transform.getAxisVector(index);		} // This is a convenience function to allow access to the axis vectors in the transform for this primitive.
	};
	
	// Represents a rigid body that can be treated as a sphere for collision detection.
	struct CollisionSphere : public CollisionPrimitive {
		real						radius;		// The radius of the sphere.
	};

	// The plane is not a primitive: it doesn't represent another rigid body. It is used for contacts with the immovable world geometry.
	struct CollisionPlane {
		Vector3						direction;	// The plane normal
		real						offset;	// The distance of the plane from the origin.
	};

	// Represents a rigid body that can be treated as an aligned bounding box for collision detection.
	struct CollisionBox : public CollisionPrimitive {
		Vector3						halfSize;	// Holds the half-sizes of the box along each of its local axes.
	};

	// A wrapper class that holds fast intersection tests. These can be used to drive the coarse collision detection system or as an early out in the full collision tests below.
	struct IntersectionTests {
		static bool					sphereAndHalfSpace					(const CollisionSphere	& sphere	, const CollisionPlane	& plane	);
		static bool					sphereAndSphere						(const CollisionSphere	& one		, const CollisionSphere	& two	);
		static bool					boxAndBox							(const CollisionBox		& one		, const CollisionBox	& two	);
		static bool					boxAndHalfSpace						(const CollisionBox		& box		, const CollisionPlane	& plane	);
	};


	// A helper structure that contains information for the detector to use in building its contact data.
	struct CollisionData {
		Contact						* contactArray						= 0;	// Holds the base of the collision data: the first contact in the array. This is used so that the contact pointer (below) can be incremented each time a contact is detected, while this pointer points to the first contact found.
		Contact						* contacts							= 0;	// Holds the contact array to write into. 
		int							contactsLeft						= 0;	// Holds the maximum number of contacts the array can take.
		uint32_t					contactCount						= 0;	// Holds the number of contacts found so far. 
		real						friction							= 0;	// Holds the friction value to write into any collisions.
		real						restitution							= 0;	// Holds the restitution value to write into any collisions.
		real						tolerance							= 0;	// Holds the collision tolerance, even uncolliding objects this close should have collisions generated.

		inline constexpr bool		hasMoreContacts						()							const		{ return contactsLeft > 0; }	// Checks if there are more contacts available in the contact data.

		void						reset								(uint32_t maxContacts)					{	// Resets the data so that it has no used contacts recorded.
			contactsLeft				= maxContacts;
			contactCount				= 0;
			contacts					= contactArray;
		}

		void						AddContacts							(uint32_t count)						{	// Notifies the data that the given number of contacts have been added.
			// Reduce the number of contacts remaining, add number used
			contactsLeft				-= count;
			contactCount				+= count;

			contacts					+= count;	// Move the array forward
		}
	};

	// A wrapper class that holds the fine grained collision detection routines. Each of the functions has the same format: it takes the details of two objects, and a pointer to a contact array to fill. It returns the number of contacts it wrote into the array.
	struct CollisionDetector {
		static uint32_t				sphereAndHalfSpace					(const CollisionSphere	& sphere	, const CollisionPlane	& plane	, CollisionData *data);
		static uint32_t				sphereAndTruePlane					(const CollisionSphere	& sphere	, const CollisionPlane	& plane	, CollisionData *data);
		static uint32_t				sphereAndSphere						(const CollisionSphere	& one		, const CollisionSphere	& two	, CollisionData *data);
		// Does a collision test on a collision box and a plane representing a half-space (i.e. the normal of the plane points out of the half-space).
		static uint32_t				boxAndHalfSpace						(const CollisionBox		& box		, const CollisionPlane	& plane	, CollisionData *data);
		static uint32_t				boxAndBox							(const CollisionBox		& one		, const CollisionBox	& two	, CollisionData *data);
		static uint32_t				boxAndPoint							(const CollisionBox		& box		, const Vector3			& point	, CollisionData *data);
		static uint32_t				boxAndSphere						(const CollisionBox		& box		, const CollisionSphere & sphere, CollisionData *data);
	};
} // namespace cyclone

#endif // CYCLONE_COLLISION_FINE_H
