// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include "collide_coarse.h"

using namespace cyclone;

BoundingSphere::BoundingSphere(const Vector3 &centre, double radius)
{
    BoundingSphere::centre = centre;
    BoundingSphere::radius = radius;
}

BoundingSphere::BoundingSphere(const BoundingSphere &one,
                               const BoundingSphere &two)
{
    Vector3 centreOffset = two.centre - one.centre;
    double distance = centreOffset.squareMagnitude();
    double radiusDiff = two.radius - one.radius;

    // Check if the larger sphere encloses the small one
    if (radiusDiff*radiusDiff >= distance)
    {
        if (one.radius > two.radius)
        {
            centre = one.centre;
            radius = one.radius;
        }
        else
        {
            centre = two.centre;
            radius = two.radius;
        }
    }

    // Otherwise we need to work with partially
    // overlapping spheres
    else
    {
        distance = real_sqrt(distance);
        radius = (distance + one.radius + two.radius) * 0.5;

        // The new centre is based on one's centre, moved towards
        // two's centre by an ammount proportional to the spheres'
        // radii.
        centre = one.centre;
        if (distance > 0)
        {
            centre += centreOffset * ((radius - one.radius)/distance);
        }
    }

}

bool BoundingSphere::overlaps(const BoundingSphere *other) const
{
    double distanceSquared = (centre - other->centre).squareMagnitude();
    return distanceSquared < (radius+other->radius)*(radius+other->radius);
}

double BoundingSphere::getGrowth(const BoundingSphere &other) const
{
    BoundingSphere newSphere(*this, other);

    // We return a value proportional to the change in surface
    // area of the sphere.
    return newSphere.radius*newSphere.radius - radius*radius;
}