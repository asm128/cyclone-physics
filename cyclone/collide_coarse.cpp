// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include "collide_coarse.h"

using namespace cyclone;

BoundingSphere::BoundingSphere(const BoundingSphere &one, const BoundingSphere &two)
{
	Vector3						centreOffset = two.Centre - one.Centre;
	double distance			= centreOffset.squareMagnitude();
	double radiusDiff		= two.Radius - one.Radius;

	if (radiusDiff*radiusDiff >= distance) {	// Check if the larger sphere encloses the small one
		if (one.Radius > two.Radius) {
			Centre					= one.Centre;
			Radius					= one.Radius;
		}
		else {
			Centre					= two.Centre;
			Radius					= two.Radius;
		}
	}
	else {	// Otherwise we need to work with partially overlapping spheres
		distance				= real_sqrt(distance);
		Radius					= (distance + one.Radius + two.Radius) * 0.5;

		// The new centre is based on one's centre, moved towards two's centre by an ammount proportional to the spheres' radii.
		Centre					= one.Centre;
		if (distance > 0)
			Centre += centreOffset * ((Radius - one.Radius)/distance);
	}

}

bool BoundingSphere::Overlaps(const BoundingSphere *other) const {
	double distanceSquared = (Centre - other->Centre).squareMagnitude();
	return distanceSquared < (Radius + other->Radius) * (Radius + other->Radius);
}

double BoundingSphere::GetGrowth(const BoundingSphere &other) const {
	BoundingSphere newSphere(*this, other);
	return newSphere.Radius * newSphere.Radius - Radius * Radius;	// We return a value proportional to the change in surface area of the sphere.
}