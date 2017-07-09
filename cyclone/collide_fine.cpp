// Implementation file for the fine grained collision detector.
// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include "collide_fine.h"

#include <memory.h>
#include <assert.h>
#include <cstdlib>
#include <cstdio>

using namespace cyclone;

bool IntersectionTests::sphereAndHalfSpace(
    const CollisionSphere &sphere,
    const CollisionPlane &plane)
{
    // Find the distance from the origin
    real ballDistance =
        plane.Direction *
        sphere.GetAxis(3) -
        sphere.Radius;

    // Check for the intersection
    return ballDistance <= plane.Offset;
}

bool IntersectionTests::sphereAndSphere(const CollisionSphere &one, const CollisionSphere &two) 
{
	Vector3 midline = one.GetAxis(3) - two.GetAxis(3);	// Find the vector between the objects
    return midline.squareMagnitude() < (one.Radius + two.Radius) * (one.Radius + two.Radius);	// See if it is large enough.
}

static inline real transformToAxis(const CollisionBox &box, const Vector3 &axis) {
    return
        box.HalfSize.x * real_abs(axis * box.GetAxis(0)) +
        box.HalfSize.y * real_abs(axis * box.GetAxis(1)) +
        box.HalfSize.z * real_abs(axis * box.GetAxis(2));
}

// This function checks if the two boxes overlap along the given axis. The final parameter toCentre is used to pass in the vector between the boxes centre points, to avoid having to recalculate it each time.
static inline bool overlapOnAxis(
    const CollisionBox &one,
    const CollisionBox &two,
    const Vector3 &axis,
    const Vector3 &toCentre
    )
{
    // Project the half-size of one onto axis
    double		oneProject		= transformToAxis(one, axis);
    double		twoProject		= transformToAxis(two, axis);

    double		distance		= real_abs(toCentre * axis);	// Project this onto the axis
    return (distance < oneProject + twoProject);	// Check for overlap
}

// This preprocessor definition is only used as a convenience
// in the boxAndBox intersection  method.
#define TEST_OVERLAP(axis) overlapOnAxis(one, two, (axis), toCentre)

bool IntersectionTests::boxAndBox(const CollisionBox &one, const CollisionBox &two) {
    Vector3 toCentre = two.GetAxis(3) - one.GetAxis(3);	// Find the vector between the two centres

    return (
        // Check on box one's axes first
        TEST_OVERLAP(one.GetAxis(0)) &&
        TEST_OVERLAP(one.GetAxis(1)) &&
        TEST_OVERLAP(one.GetAxis(2)) &&

        // And on two's
        TEST_OVERLAP(two.GetAxis(0)) &&
        TEST_OVERLAP(two.GetAxis(1)) &&
        TEST_OVERLAP(two.GetAxis(2)) &&

        // Now on the croSs products
        TEST_OVERLAP(one.GetAxis(0) % two.GetAxis(0)) &&
        TEST_OVERLAP(one.GetAxis(0) % two.GetAxis(1)) &&
        TEST_OVERLAP(one.GetAxis(0) % two.GetAxis(2)) &&
        TEST_OVERLAP(one.GetAxis(1) % two.GetAxis(0)) &&
        TEST_OVERLAP(one.GetAxis(1) % two.GetAxis(1)) &&
        TEST_OVERLAP(one.GetAxis(1) % two.GetAxis(2)) &&
        TEST_OVERLAP(one.GetAxis(2) % two.GetAxis(0)) &&
        TEST_OVERLAP(one.GetAxis(2) % two.GetAxis(1)) &&
        TEST_OVERLAP(one.GetAxis(2) % two.GetAxis(2))
    );
}
#undef TEST_OVERLAP

bool IntersectionTests::boxAndHalfSpace(
    const CollisionBox &box,
    const CollisionPlane &plane
    )
{
    // Work out the projected radius of the box onto the plane direction
    real projectedRadius = transformToAxis(box, plane.Direction);

    // Work out how far the box is from the origin
    real boxDistance =
        plane.Direction *
        box.GetAxis(3) -
        projectedRadius;

    // Check for the intersection
    return boxDistance <= plane.Offset;
}

unsigned CollisionDetector::sphereAndTruePlane(
    const CollisionSphere &sphere,
    const CollisionPlane &plane,
    CollisionData *data
    )
{
    // Make sure we have contacts
    if (data->contactsLeft <= 0) return 0;

    // Cache the sphere position
    Vector3 position = sphere.GetAxis(3);

    // Find the distance from the plane
    real centreDistance = plane.Direction * position - plane.Offset;

    // Check if we're within radius
    if (centreDistance*centreDistance > sphere.Radius*sphere.Radius)
    {
        return 0;
    }

    // Check which side of the plane we're on
    Vector3 normal = plane.Direction;
    real penetration = -centreDistance;
    if (centreDistance < 0)
    {
        normal *= -1;
        penetration = -penetration;
    }
    penetration += sphere.Radius;

    // Create the contact - it has a normal in the plane direction.
    Contact* contact = data->contacts;
    contact->ContactNormal	= normal;
    contact->Penetration	= penetration;
    contact->ContactPoint	= position - plane.Direction * centreDistance;
    contact->setBodyData(sphere.Body, NULL,
        data->friction, data->restitution);

    data->AddContacts(1);
    return 1;
}

unsigned CollisionDetector::sphereAndHalfSpace(
    const CollisionSphere &sphere,
    const CollisionPlane &plane,
    CollisionData *data
    )
{
    if (data->contactsLeft <= 0)	// Make sure we have contacts
		return 0;

	Vector3 position = sphere.GetAxis(3);	// Cache the sphere position
    real	ballDistance = plane.Direction * position - sphere.Radius - plane.Offset;	// Find the distance from the plane
    if (ballDistance >= 0) 
		return 0;

    // Create the contact - it has a normal in the plane direction.
    Contact* contact = data->contacts;
    contact->ContactNormal	= plane.Direction;
    contact->Penetration	= -ballDistance;
    contact->ContactPoint	=
        position - plane.Direction * (ballDistance + sphere.Radius);
    contact->setBodyData(sphere.Body, NULL, data->friction, data->restitution);

    data->AddContacts(1);
    return 1;
}

unsigned CollisionDetector::sphereAndSphere(
    const CollisionSphere &one,
    const CollisionSphere &two,
    CollisionData *data
    )
{
    // Make sure we have contacts
    if (data->contactsLeft <= 0) return 0;

    // Cache the sphere positions
    Vector3 positionOne = one.GetAxis(3);
    Vector3 positionTwo = two.GetAxis(3);

    // Find the vector between the objects
    Vector3							midline				= positionOne - positionTwo;
    real							size				= midline.magnitude();

    if (size <= 0.0f || size >= one.Radius + two.Radius)	// See if it is large enough.
        return 0;

    // We manually create the normal, because we have the size to hand.
    Vector3							normal				= midline * (((real)1.0)/size);
    Contact							* contact			= data->contacts;
    contact->ContactNormal		= normal;
    contact->ContactPoint		= positionOne + midline * (real)0.5;
    contact->Penetration		= (one.Radius + two.Radius - size);
    contact->setBodyData(one.Body, two.Body,
        data->friction, data->restitution);

    data->AddContacts(1);
    return 1;
}

// This function checks if the two boxes overlap along the given axis, returning the ammount of overlap.
// The final parameter toCentre is used to pass in the vector between the boxes centre points, to avoid having to recalculate it each time.
static inline real penetrationOnAxis(
    const CollisionBox &one,
    const CollisionBox &two,
    const Vector3 &axis,
    const Vector3 &toCentre
    )
{
    // Project the half-size of one onto axis
    double		oneProject	= transformToAxis(one, axis);
    double		twoProject	= transformToAxis(two, axis);

    double		distance	= real_abs(toCentre * axis);	// Project this onto the axis
    return oneProject + twoProject - distance;				// Return the overlap (i.e. positive indicates overlap, negative indicates separation).
}


static inline bool tryAxis(
    const CollisionBox		& one
    , const CollisionBox	& two
    , Vector3				axis
    , const Vector3			& toCentre
    , uint32_t index	
    // These values may be updated
    , real					& smallestPenetration
    , uint32_t				& smallestCase
    )
{
    // Make sure we have a normalized axis, and don't check almost parallel axes
    if (axis.squareMagnitude() < 0.0001) return true;
    axis.normalise();

    real penetration = penetrationOnAxis(one, two, axis, toCentre);

    if (penetration < 0) return false;
    if (penetration < smallestPenetration) {
        smallestPenetration = penetration;
        smallestCase = index;
    }
    return true;
}

void fillPointFaceBoxBox
	( const CollisionBox	& one
	, const CollisionBox	& two
	, const Vector3			& toCentre
	, CollisionData			* data
	, uint32_t				best
	, real					pen
	)
{
    Contact									* contact					= data->contacts;	// This method is called when we know that a vertex from box two is in contact with box one.

    // We know which axis the collision is on (i.e. best), but we need to work out which of the two faces on this axis.
    Vector3									normal						= one.GetAxis(best);
    if (one.GetAxis(best) * toCentre > 0)
        normal								= normal * -1.0f;

    // Work out which vertex of box two we're colliding with.
    // Using toCentre doesn't work!
    Vector3									vertex						= two.HalfSize;
    if (two.GetAxis(0) * normal < 0) vertex.x = -vertex.x;
    if (two.GetAxis(1) * normal < 0) vertex.y = -vertex.y;
    if (two.GetAxis(2) * normal < 0) vertex.z = -vertex.z;

    // Create the contact data
    contact->ContactNormal				= normal;
    contact->Penetration				= pen;
    contact->ContactPoint				= two.Transform * vertex;
    contact->setBodyData(one.Body, two.Body, data->friction, data->restitution);
}

static inline Vector3 contactPoint(
    const Vector3 &pOne,
    const Vector3 &dOne,
    real oneSize,
    const Vector3 &pTwo,
    const Vector3 &dTwo,
    real twoSize,

    // If this is true, and the contact point is outside
    // the edge (in the case of an edge-face contact) then
    // we use one's midpoint, otherwise we use two's.
    bool useOne)
{
    Vector3 toSt, cOne, cTwo;
    real dpStaOne, dpStaTwo, dpOneTwo, smOne, smTwo;
    real denom, mua, mub;

    smOne = dOne.squareMagnitude();
    smTwo = dTwo.squareMagnitude();
    dpOneTwo = dTwo * dOne;

    toSt = pOne - pTwo;
    dpStaOne = dOne * toSt;
    dpStaTwo = dTwo * toSt;

    denom = smOne * smTwo - dpOneTwo * dpOneTwo;

    // Zero denominator indicates parrallel lines
    if (real_abs(denom) < 0.0001f) {
        return useOne?pOne:pTwo;
    }

    mua = (dpOneTwo * dpStaTwo - smTwo * dpStaOne) / denom;
    mub = (smOne * dpStaTwo - dpOneTwo * dpStaOne) / denom;

    // If either of the edges has the nearest point out
    // of bounds, then the edges aren't crossed, we have
    // an edge-face contact. Our point is on the edge, which
    // we know from the useOne parameter.
    if (mua > oneSize ||
        mua < -oneSize ||
        mub > twoSize ||
        mub < -twoSize)
    {
        return useOne?pOne:pTwo;
    }
    else
    {
        cOne = pOne + dOne * mua;
        cTwo = pTwo + dTwo * mub;

        return cOne * 0.5 + cTwo * 0.5;
    }
}

// This preprocessor definition is only used as a convenience
// in the boxAndBox contact generation method.
#define CHECK_OVERLAP(axis, index) \
    if (!tryAxis(one, two, (axis), toCentre, (index), pen, best)) return 0;

unsigned CollisionDetector::boxAndBox(
    const CollisionBox &one,
    const CollisionBox &two,
    CollisionData *data
    )
{
	//if (!IntersectionTests::boxAndBox(one, two)) return 0;
	Vector3				toCentre	= two.GetAxis(3) - one.GetAxis(3);	// Find the vector between the two centres
	
	// We start assuming there is no contact
	real				pen			= REAL_MAX;
	uint32_t			best		= 0xffffff;
	
	// Now we check each axes, returning if it gives us a separating axis, and keeping track of the axis with the smallest penetration otherwise.
	CHECK_OVERLAP(one.GetAxis(0), 0);
	CHECK_OVERLAP(one.GetAxis(1), 1);
	CHECK_OVERLAP(one.GetAxis(2), 2);
	
	CHECK_OVERLAP(two.GetAxis(0), 3);
	CHECK_OVERLAP(two.GetAxis(1), 4);
	CHECK_OVERLAP(two.GetAxis(2), 5);
	
	// Store the best axis-major, in case we run into almost parallel edge collisions later
	unsigned bestSingleAxis = best;
	
	CHECK_OVERLAP(one.GetAxis(0) % two.GetAxis(0), 6);
	CHECK_OVERLAP(one.GetAxis(0) % two.GetAxis(1), 7);
	CHECK_OVERLAP(one.GetAxis(0) % two.GetAxis(2), 8);
	CHECK_OVERLAP(one.GetAxis(1) % two.GetAxis(0), 9);
	CHECK_OVERLAP(one.GetAxis(1) % two.GetAxis(1), 10);
	CHECK_OVERLAP(one.GetAxis(1) % two.GetAxis(2), 11);
	CHECK_OVERLAP(one.GetAxis(2) % two.GetAxis(0), 12);
	CHECK_OVERLAP(one.GetAxis(2) % two.GetAxis(1), 13);
	CHECK_OVERLAP(one.GetAxis(2) % two.GetAxis(2), 14);
	
	if(best == 0xffffff)
		throw("Make sure we've got a result.");
	
	// We now know there's a collision, and we know which of the axes gave the smallest penetration. We now can deal with it in different ways depending on the case.
	if (best < 3) { // We've got a vertex of box two on a face of box one.
        fillPointFaceBoxBox(one, two, toCentre, data, best, pen);
        data->AddContacts(1);
        return 1;
    }
    else if (best < 6) { // We've got a vertex of box one on a face of box two. We use the same algorithm as above, but swap around one and two (and therefore also the vector between their centres).
        fillPointFaceBoxBox(two, one, toCentre*-1.0f, data, best-3, pen);
        data->AddContacts(1);
        return 1;
    }
    else
    {
        // We've got an edge-edge contact. Find out which axes
        best -= 6;
        unsigned oneAxisIndex = best / 3;
        unsigned twoAxisIndex = best % 3;
        Vector3 oneAxis = one.GetAxis(oneAxisIndex);
        Vector3 twoAxis = two.GetAxis(twoAxisIndex);
        Vector3 axis = oneAxis % twoAxis;
        axis.normalise();

        // The axis should point from box one to box two.
        if (axis * toCentre > 0) axis = axis * -1.0f;

        // We have the axes, but not the edges: each axis has 4 edges parallel to it, we need to find which of the 4 for each object. We do that by finding the point in the centre of the edge. 
		// We know its component in the direction of the box's collision axis is zero (its a mid-point) and we determine which of the extremes in each of the other axes is closest.
        Vector3 ptOnOneEdge = one.HalfSize;
        Vector3 ptOnTwoEdge = two.HalfSize;
        for (unsigned i = 0; i < 3; i++)
        {
					if (i == oneAxisIndex)			ptOnOneEdge[i] = 0;
            else	if (one.GetAxis(i) * axis > 0)	ptOnOneEdge[i] = -ptOnOneEdge[i];

            if (i == twoAxisIndex) ptOnTwoEdge[i] = 0;
            else if (two.GetAxis(i) * axis < 0) ptOnTwoEdge[i] = -ptOnTwoEdge[i];
        }

        // Move them into world coordinates (they are already oriented correctly, since they have been derived from the axes).
        ptOnOneEdge = one.Transform * ptOnOneEdge;
        ptOnTwoEdge = two.Transform * ptOnTwoEdge;

        // So we have a point and a direction for the colliding edges. We need to find out point of closest approach of the two line-segments.
        Vector3 vertex = contactPoint(
            ptOnOneEdge, oneAxis, one.HalfSize[oneAxisIndex],
            ptOnTwoEdge, twoAxis, two.HalfSize[twoAxisIndex],
            bestSingleAxis > 2
            );

        // We can fill the contact.
        Contact* contact = data->contacts;

        contact->Penetration = pen;
        contact->ContactNormal = axis;
        contact->ContactPoint = vertex;
        contact->setBodyData(one.Body, two.Body,
            data->friction, data->restitution);
        data->AddContacts(1);
        return 1;
    }
}
#undef CHECK_OVERLAP




unsigned CollisionDetector::boxAndPoint(
    const CollisionBox &box,
    const Vector3 &point,
    CollisionData *data
    )
{
    Vector3 relPt	= box.Transform.transformInverse(point);	// Transform the point into box coordinates
    Vector3 normal;

    // Check each axis, looking for the axis on which the penetration is least deep.
    real min_depth = box.HalfSize.x - real_abs(relPt.x);
    if (min_depth < 0) 
		return 0;
    normal = box.GetAxis(0) * ((relPt.x < 0)?-1:1);

    real depth = box.HalfSize.y - real_abs(relPt.y);
    if (depth < 0) 
		return 0;
    else if (depth < min_depth) {
        min_depth = depth;
        normal = box.GetAxis(1) * ((relPt.y < 0)?-1:1);
    }

    depth = box.HalfSize.z - real_abs(relPt.z);
		 if (depth < 0) 
			 return 0;
    else if (depth < min_depth) {
        min_depth	= depth;
        normal		= box.GetAxis(2) * ((relPt.z < 0)?-1:1);
    }

    // Compile the contact
    Contact* contact = data->contacts;
    contact->ContactNormal = normal;
    contact->ContactPoint = point;
    contact->Penetration = min_depth;

    // Note that we don't know what rigid body the point
    // belongs to, so we just use NULL. Where this is called
    // this value can be left, or filled in.
    contact->setBodyData(box.Body, NULL,
        data->friction, data->restitution);

    data->AddContacts(1);
    return 1;
}

unsigned CollisionDetector::boxAndSphere(
    const CollisionBox &box,
    const CollisionSphere &sphere,
    CollisionData *data
    )
{
    // Transform the centre of the sphere into box coordinates
    Vector3 centre = sphere.GetAxis(3);
    Vector3 relCentre = box.Transform.transformInverse(centre);

    // Early out check to see if we can exclude the contact
    if (real_abs(relCentre.x) - sphere.Radius > box.HalfSize.x ||
        real_abs(relCentre.y) - sphere.Radius > box.HalfSize.y ||
        real_abs(relCentre.z) - sphere.Radius > box.HalfSize.z)
    {
        return 0;
    }

	Vector3 closestPt		= {};
    real dist;

    // Clamp each coordinate to the box.
    dist = relCentre.x;
    if (dist > box.HalfSize.x) dist = box.HalfSize.x;
    if (dist < -box.HalfSize.x) dist = -box.HalfSize.x;
    closestPt.x = dist;

    dist = relCentre.y;
    if (dist > box.HalfSize.y) dist = box.HalfSize.y;
    if (dist < -box.HalfSize.y) dist = -box.HalfSize.y;
    closestPt.y = dist;

    dist = relCentre.z;
    if (dist > box.HalfSize.z) dist = box.HalfSize.z;
    if (dist < -box.HalfSize.z) dist = -box.HalfSize.z;
    closestPt.z = dist;

    // Check we're in contact
    dist = (closestPt - relCentre).squareMagnitude();
    if (dist > sphere.Radius * sphere.Radius) return 0;

    // Compile the contact
    Vector3 closestPtWorld = box.Transform.transform(closestPt);

    Contact* contact = data->contacts;
    contact->ContactNormal = (closestPtWorld - centre);
    contact->ContactNormal.normalise();
    contact->ContactPoint = closestPtWorld;
    contact->Penetration = sphere.Radius - real_sqrt(dist);
    contact->setBodyData(box.Body, sphere.Body,
        data->friction, data->restitution);

    data->AddContacts(1);
    return 1;
}

unsigned CollisionDetector::boxAndHalfSpace(
    const CollisionBox &box,
    const CollisionPlane &plane,
    CollisionData *data
    )
{
	if (data->contactsLeft <= 0)		// Make sure we have contacts
		return 0;

	if (!IntersectionTests::boxAndHalfSpace(box, plane))	// Check for intersection
	    return 0;

	// --- We have an intersection, so find the intersection points. We can make do with only checking vertices. If the box is resting on a plane or on an edge, it will be reported as four or two contact points.

	// Go through each combination of + and - for each half-size
	static real mults[8][3] = {{1,1,1},{-1,1,1},{1,-1,1},{-1,-1,1},
	                           {1,1,-1},{-1,1,-1},{1,-1,-1},{-1,-1,-1}};

	Contact* contact = data->contacts;
	unsigned contactsUsed = 0;
	for (unsigned i = 0; i < 8; i++) {

	    // Calculate the position of each vertex
		Vector3 vertexPos	= {mults[i][0], mults[i][1], mults[i][2]};
	    vertexPos.componentProductUpdate(box.HalfSize);
	    vertexPos = box.Transform.transform(vertexPos);

	    real							vertexDistance							= vertexPos * plane.Direction;	// Calculate the distance from the plane

	    if (vertexDistance <= plane.Offset) {	// Compare this to the plane's distance
			// Create the contact data. The contact point is halfway between the vertex and the plane - we multiply the direction by half the separation distance and add the vertex location.
			contact->ContactPoint		= plane.Direction;
			contact->ContactPoint		*= vertexDistance - plane.Offset;
			contact->ContactPoint		+= vertexPos;
			contact->ContactNormal		= plane.Direction;
			contact->Penetration		= plane.Offset - vertexDistance;

			contact->setBodyData(box.Body, NULL, data->friction, data->restitution);	// Write the appropriate data

	        // Move onto the next contact
	        ++contact;
	        ++contactsUsed;
	        if (contactsUsed == (unsigned)data->contactsLeft) return contactsUsed;
	    }
	}

	data->AddContacts(contactsUsed);
	return contactsUsed;
}
