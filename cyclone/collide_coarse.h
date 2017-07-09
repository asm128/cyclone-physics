// This file contains the coarse collision detection system. It is used to return pairs of objects that may be in contact, which can then be tested using fined grained methods.
// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include "contacts.h"

#include <vector>
#include <cstddef>

#ifndef CYCLONE_COLLISION_COARSE_H
#define CYCLONE_COLLISION_COARSE_H

namespace cyclone {

	// Represents a bounding sphere that can be tested for overlap.
	struct BoundingSphere {
		Vector3						centre;
		double						radius;
	public:
									BoundingSphere				(const Vector3 &centre, double radius);
									BoundingSphere				(const BoundingSphere &one, const BoundingSphere &two);	// Creates a bounding sphere to enclose the two given bounding spheres.

		bool						overlaps					(const BoundingSphere *other)									const;	// Checks if the bounding sphere overlaps with the other given bounding sphere.
		// Reports how much this bounding sphere would have to grow by to incorporate the given bounding sphere. Note that this calculation returns a value not in any particular units (i.e. its not a volume growth). 
		// In fact the best implementation takes into account the growth in surface area (after the * Goldsmith-Salmon algorithm for tree construction).
		double						getGrowth					(const BoundingSphere &other)									const;
			
		// Returns the volume of this bounding volume. This is used to calculate how to recurse into the bounding volume tree. For a bounding sphere it is a simple calculation.
		double						getSize						()																const		{ return 1.333333 * R_PI * radius * radius * radius; }
	};

	// Stores a potential contact to check later.
	struct PotentialContact {
		RigidBody *					body		[2]				= {};	// Holds the bodies that might be in contact.
	};

	// A base class for nodes in a bounding volume hierarchy. This class uses a binary tree to store the bounding volumes.
	template<class BoundingVolumeClass>
	struct BVHNode {
		BVHNode *					children	[2]				= {};	// Holds the child nodes of this node.
		BoundingVolumeClass			volume						= {};	// Holds a single bounding volume encompassing all the descendents of this node.

		// Holds the rigid body at this node of the hierarchy. Only leaf nodes can have a rigid body defined (see isLeaf).
		// Note that it is possible to rewrite the algorithms in this class to handle objects at all levels of the hierarchy,
		// but the code provided ignores this vector unless firstChild is NULL.
		RigidBody					* body						= 0;
		BVHNode						* parent					= 0;	// Holds the node immediately above us in the tree.

		// Creates a new node in the hierarchy with the given parameters.
									BVHNode						(BVHNode *parent, const BoundingVolumeClass &volume, RigidBody* body=NULL)	: parent(parent), volume(volume), body(body)				{}
		
		// Checks if this node is at the bottom of the hierarchy.
		bool						isLeaf						()																															const		{ return (body != NULL); }
		
		// Checks the potential contacts from this node downwards in the hierarchy, writing them to the given array (up to the given limit). Returns the number of potential contacts it found.
		uint32_t					getPotentialContacts		(PotentialContact* contacts, uint32_t limit)																				const		{
			if (isLeaf() || limit == 0)		// Early out if we don't have the room for contacts, or if we're a leaf node.
				return 0;
			return children[0]->getPotentialContactsWith(children[1], contacts, limit);	// Get the potential contacts of one of our children with the other
		}
		void						insert						(RigidBody* body, const BoundingVolumeClass &volume);	// Inserts the given rigid body, with the given bounding volume, into the hierarchy. This may involve the creation of further bounding volume nodes.

		// Deletes this node, removing it first from the hierarchy, along with its associated rigid body and child nodes. This method deletes the node and all its children (but obviously not the rigid bodies). 
		// This also has the effect of deleting the sibling of this node, and changing the parent node so that it contains the data currently in that sibling. Finally it forces the hierarchy above the current node to reconsider its bounding volume.
									~BVHNode					();
	protected:
		// Checks for overlapping between nodes in the hierarchy. Note that any bounding volume should have an overlaps method implemented that checks for overlapping with another object of its own type.
		bool						overlaps					(const BVHNode<BoundingVolumeClass> *other)																					const			{ return volume->overlaps(other->volume); }	
		uint32_t					getPotentialContactsWith	(const BVHNode<BoundingVolumeClass> *other, PotentialContact* contacts, uint32_t limit) const;	// Checks the potential contacts between this node and the given other node, writing them to the given array (up to the given limit). Returns the number of potential contacts it found.
		void						recalculateBoundingVolume	(bool recurse = true);	// For non-leaf nodes, this method recalculates the bounding volume based on the bounding volumes of its children.
	};

	template<class BoundingVolumeClass>
				BVHNode<BoundingVolumeClass>::~BVHNode						()																									{
		// If we don't have a parent, then we ignore the sibling processing
		if (parent) {	// Find our sibling
			BVHNode<BoundingVolumeClass>		*sibling	= 0;
			if (parent->children[0] == this) 
				sibling = parent->children[1];
			else 
				sibling = parent->children[0];

			// Write its data to our parent
			parent->volume			= sibling->volume;
			parent->body			= sibling->body;
			parent->children[0]		= sibling->children[0];
			parent->children[1]		= sibling->children[1];

			// Delete the sibling (we blank its parent and children to avoid processing/deleting them)
			*sibling				= {};
			sibling->parent			= NULL;
			sibling->body			= NULL;
			sibling->children[0]	= NULL;
			sibling->children[1]	= NULL;
			delete sibling;

			parent->recalculateBoundingVolume();	// Recalculate the parent's bounding volume
		}

		// Delete our children (again we remove their parent data so we don't try to process their siblings as they are deleted).
		if (children[0]) {
			children[0]->parent		= NULL;
			delete children[0];
		}
		if (children[1]) {
			children[1]->parent		= NULL;
			delete children[1];
		}
	}

	template<class BoundingVolumeClass>
	void		BVHNode<BoundingVolumeClass>::insert						(RigidBody* newBody, const BoundingVolumeClass &newVolume)											{
		if (isLeaf()) {	// If we are a leaf, then the only option is to spawn two new children and place the new body in one.
			children[0]				= new BVHNode<BoundingVolumeClass>(this, volume, body);			// Child one is a copy of us.
			children[1]				= new BVHNode<BoundingVolumeClass>(this, newVolume, newBody);	// Child two holds the new body
			this->body				= NULL;	// And we now loose the body (we're no longer a leaf)

			recalculateBoundingVolume();	// We need to recalculate our bounding volume
		}
		else {	// Otherwise we need to work out which child gets to keep the inserted body. We give it to whoever would grow the least to incorporate it.
			if (children[0]->volume.getGrowth(newVolume) < children[1]->volume.getGrowth(newVolume))
				children[0]->insert(newBody, newVolume);
			else
				children[1]->insert(newBody, newVolume);
		}
	}

	template<class BoundingVolumeClass>
	void		BVHNode<BoundingVolumeClass>::recalculateBoundingVolume		(bool recurse)																						{
		if(isLeaf()) 
			return;
		volume = BoundingVolumeClass(children[0]->volume, children[1]->volume);	// Use the bounding volume combining constructor.
		if(parent) // Recurse up the tree
			parent->recalculateBoundingVolume(true);
	}

	template<class BoundingVolumeClass>
	uint32_t	BVHNode<BoundingVolumeClass>::getPotentialContactsWith		(const BVHNode<BoundingVolumeClass> *other, PotentialContact* contacts, uint32_t limit)		const	{
		if (!overlaps(other) || limit == 0)		// Early out if we don't overlap or if we have no room to report contacts
			return 0;

		if (isLeaf() && other->isLeaf()) {	// If we're both at leaf nodes, then we have a potential contact
			contacts->body[0] = body;
			contacts->body[1] = other->body;
			return 1;
		}

		// Determine which node to descend into. If either is a leaf, then we descend the other. If both are branches, then we use the one with the largest size.
		if( other->isLeaf() 
		 || (!isLeaf() && volume->getSize() >= other->volume->getSize())
		 )
		{
			uint32_t count = children[0]->getPotentialContactsWith(other, contacts, limit);	// Recurse into ourself

			if (limit > count) 	// Check we have enough slots to do the other side too
				return count + children[1]->getPotentialContactsWith(other, contacts+count, limit-count);
			else 
				return count;
		}
		else {	// Recurse into the other node
			uint32_t count = getPotentialContactsWith(other->children[0], contacts, limit);

			if (limit > count) 	// Check we have enough slots to do the other side too
				return count + getPotentialContactsWith(other->children[1], contacts+count, limit-count);
			else 
				return count;
		}
	}
} // namespace cyclone

#endif // CYCLONE_COLLISION_FINE_H
