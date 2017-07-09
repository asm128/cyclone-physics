// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include "contacts.h"

#include <memory.h>
#include <assert.h>

using namespace cyclone;

// Contact implementation

void Contact::setBodyData(RigidBody* one, RigidBody *two, double friction, double restitution)
{
    Contact::Body[0]		= one;
    Contact::Body[1]		= two;
    Contact::Friction		= friction;
    Contact::Restitution	= restitution;
}

void Contact::matchAwakeState()
{
    // Collisions with the world never cause a body to wake up.
    if(!Body[1]) 
		return;

    bool body0awake = Body[0]->IsAwake;
    bool body1awake = Body[1]->IsAwake;

    // Wake up only the sleeping one
    if (body0awake ^ body1awake) {
        if (body0awake) 
			Body[1]->setAwake();
        else 
			Body[0]->setAwake();
    }
}

// Swaps the bodies in the current contact, so body 0 is at body 1 and vice versa. This also changes the direction of the contact normal, but doesn't update any calculated internal data. 
// If you are calling this method manually, then call calculateInternals afterwards to make sure the internal data is up to date.
void Contact::swapBodies() {
	ContactNormal					*= -1;
	
	RigidBody							* temp							= Body[0];
	Body[0]							= Body[1];
	Body[1]							= temp;
}

// Constructs an arbitrary orthonormal basis for the contact.  This is stored as a 3x3 matrix, where each vector is a column (in other words the matrix transforms contact space into world space). 
// The x direction is generated from the contact normal, and the y and z directionss are set so they are at right angles to it.
inline void Contact::calculateContactBasis()
{
    Vector3 contactTangent[2];

    if (real_abs(ContactNormal.x) > real_abs(ContactNormal.y)) {	// Check whether the Z-axis is nearer to the X or Y axis
        const double s = 1.0/real_sqrt(ContactNormal.z * ContactNormal.z + ContactNormal.x*ContactNormal.x);	// Scaling factor to ensure the results are normalised

        // The new X-axis is at right angles to the world Y-axis
        contactTangent[0].x = ContactNormal.z*s;
        contactTangent[0].y = 0;
        contactTangent[0].z = -ContactNormal.x*s;

        // The new Y-axis is at right angles to the new X- and Z- axes
        contactTangent[1].x =  ContactNormal.y * contactTangent[0].x;
        contactTangent[1].y =  ContactNormal.z * contactTangent[0].x - ContactNormal.x*contactTangent[0].z;
        contactTangent[1].z = -ContactNormal.y * contactTangent[0].x;
    }
    else {
		
		const double s = 1.0 / real_sqrt(ContactNormal.z*ContactNormal.z + ContactNormal.y*ContactNormal.y);	// Scaling factor to ensure the results are normalised

        // The new X-axis is at right angles to the world X-axis
        contactTangent[0].x = 0;
        contactTangent[0].y = -ContactNormal.z * s;
        contactTangent[0].z =  ContactNormal.y * s;

        // The new Y-axis is at right angles to the new X- and Z- axes
        contactTangent[1].x =  ContactNormal.y * contactTangent[0].z - ContactNormal.z*contactTangent[0].y;
        contactTangent[1].y = -ContactNormal.x * contactTangent[0].z;
        contactTangent[1].z =  ContactNormal.x * contactTangent[0].y;
    }

    // Make a matrix from the three vectors.
    ContactToWorld.setComponents(
        ContactNormal,
        contactTangent[0],
        contactTangent[1]);
}

Vector3 Contact::calculateLocalVelocity(uint32_t bodyIndex, double duration) {
    RigidBody											* thisBody						= Body[bodyIndex];
    Vector3												velocity						= thisBody->Rotation % RelativeContactPosition[bodyIndex];	// Work out the velocity of the contact point.
    velocity										+= thisBody->Velocity;
	Vector3												contactVelocity					= ContactToWorld.transformTranspose(velocity);	// Turn the velocity into contact-coordinates.
    Vector3												accVelocity						= thisBody->LastFrameAcceleration * duration;	// Calculate the ammount of velocity that is due to forces without reactions.
    accVelocity										= ContactToWorld.transformTranspose(accVelocity);	// Calculate the velocity in contact-coordinates.
    accVelocity.x									= 0;	// We ignore any component of acceleration in the contact normal direction, we are only interested in planar acceleration
    contactVelocity									+= accVelocity;	// Add the planar velocities - if there's enough friction they will be removed during velocity resolution
    return contactVelocity;	// And return it
}


void Contact::calculateDesiredDeltaVelocity(double duration) {
	const static double									velocityLimit					= (double)0.25f;
	double												velocityFromAcc					= 0;	// Calculate the acceleration induced velocity accumulated this frame

	if (Body[0]->IsAwake)
		velocityFromAcc									+= Body[0]->LastFrameAcceleration * duration * ContactNormal;

	if (Body[1] && Body[1]->IsAwake)
		velocityFromAcc									-= Body[1]->LastFrameAcceleration * duration * ContactNormal;

	double												thisRestitution					= Restitution;
	if (real_abs(ContactVelocity.x) < velocityLimit)	// If the velocity is very slow, limit the restitution
		thisRestitution									= (double)0.0f;

	DesiredDeltaVelocity							= - ContactVelocity.x - thisRestitution * (ContactVelocity.x - velocityFromAcc);	// Combine the bounce velocity with the removed acceleration velocity.
}


void Contact::calculateInternals(double duration) {
	if (!Body[0])	// Check if the first object is NULL, and swap if it is.
		swapBodies();
	
	calculateContactBasis();	// Calculate an set of axis at the contact point.
	RelativeContactPosition[0]		= ContactPoint - Body[0]->Position;	// Store the relative position of the contact relative to each body
	if (Body[1]) 
		RelativeContactPosition[1]		= ContactPoint - Body[1]->Position;
	
	ContactVelocity					= calculateLocalVelocity(0, duration);	// Find the relative velocity of the bodies at the contact point.
	if (Body[1]) {
		ContactVelocity					-= calculateLocalVelocity(1, duration);
	}
	calculateDesiredDeltaVelocity(duration);		// Calculate the desired change in velocity for resolution
}

void Contact::applyVelocityChange	(Vector3 velocityChange[2]
									,Vector3 rotationChange[2]
									)
{
    // Get hold of the inverse mass and inverse inertia tensor, both in world coordinates.
    Matrix3 inverseInertiaTensor[2];
    inverseInertiaTensor[0] = Body[0]->InverseInertiaTensorWorld;
    if (Body[1])
        inverseInertiaTensor[1] = Body[1]->InverseInertiaTensorWorld;

    // We will calculate the impulse for each contact axis
    Vector3 impulseContact;

	if (Friction == (double)0.0)
		impulseContact			= calculateFrictionlessImpulse(inverseInertiaTensor);	// Use the short format for frictionless contacts
	else {
		impulseContact			= calculateFrictionImpulse(inverseInertiaTensor);	// Otherwise we may have impulses that aren't in the direction of the contact, so we need the more complex version.
	}

	Vector3						impulse				= ContactToWorld.transform(impulseContact);	// Convert impulse to world coordinates
    Vector3						impulsiveTorque0		= RelativeContactPosition[0] % impulse;	// Split in the impulse into linear and rotational components
    rotationChange[0] = inverseInertiaTensor[0].transform(impulsiveTorque0);
    velocityChange[0].clear();
    velocityChange[0].addScaledVector(impulse, Body[0]->InverseMass);

    // Apply the changes
    Body[0]->Velocity		+= velocityChange[0];
    Body[0]->Rotation		+= rotationChange[0];

    if (Body[1]) {	// Work out body one's linear and angular changes
        Vector3						impulsiveTorque1		= impulse % RelativeContactPosition[1];
        rotationChange[1]		= inverseInertiaTensor[1].transform(impulsiveTorque1);
        velocityChange[1].clear();
        velocityChange[1].addScaledVector(impulse, - Body[1]->InverseMass);

        // And apply them.
        Body[1]->Velocity		+= velocityChange[1];
        Body[1]->Rotation		+= rotationChange[1];
    }
}

inline Vector3 Contact::calculateFrictionlessImpulse(Matrix3 * inverseInertiaTensor) {
    Vector3							impulseContact;

    // Build a vector that shows the change in velocity in world space for a unit impulse in the direction of the contact normal.
    Vector3							deltaVelWorld0							= RelativeContactPosition[0] % ContactNormal;
    deltaVelWorld0				= inverseInertiaTensor[0].transform(deltaVelWorld0);
    deltaVelWorld0				= deltaVelWorld0 % RelativeContactPosition[0];

    double							deltaVelocity							= deltaVelWorld0 * ContactNormal;	// Work out the change in velocity in contact coordiantes.
    deltaVelocity				+= Body[0]->InverseMass;	// Add the linear component of velocity change
	if (Body[1]) {	// Check if we need to the second body's data
        
		// Go through the same transformation sequence again
        Vector3							deltaVelWorld1 = RelativeContactPosition[1] % ContactNormal;
        deltaVelWorld1				= inverseInertiaTensor[1].transform(deltaVelWorld1);
        deltaVelWorld1				= deltaVelWorld1 % RelativeContactPosition[1];

        deltaVelocity				+= deltaVelWorld1 * ContactNormal;	// Add the change in velocity due to rotation
        deltaVelocity				+= Body[1]->InverseMass;	// Add the change in velocity due to linear motion
    }
    // Calculate the required size of the impulse
    impulseContact.x			= DesiredDeltaVelocity / deltaVelocity;
    impulseContact.y			= 0;
    impulseContact.z			= 0;
    return impulseContact;
}

inline Vector3 Contact::calculateFrictionImpulse(Matrix3 * inverseInertiaTensor) {
    double							inverseMass			= Body[0]->InverseMass;
    Matrix3							impulseToTorque;
    impulseToTorque.setSkewSymmetric(RelativeContactPosition[0]);	// The equivalent of a cross product in matrices is multiplication by a skew symmetric matrix - we build the matrix for converting between linear and angular quantities.

    // Build the matrix to convert contact impulse to change in velocity
    // in world coordinates.
    Matrix3 deltaVelWorld = impulseToTorque;
    deltaVelWorld *= inverseInertiaTensor[0];
    deltaVelWorld *= impulseToTorque;
    deltaVelWorld *= -1;

    // Check if we need to add body two's data
    if (Body[1])
    {
        // Set the cross product matrix
        impulseToTorque.setSkewSymmetric(RelativeContactPosition[1]);

        // Calculate the velocity change matrix
        Matrix3					deltaVelWorld2				= impulseToTorque;
        deltaVelWorld2		*= inverseInertiaTensor[1];
        deltaVelWorld2		*= impulseToTorque;
        deltaVelWorld2		*= -1;

        deltaVelWorld		+= deltaVelWorld2;			// Add to the total delta velocity.
        inverseMass			+= Body[1]->InverseMass;	// Add to the inverse mass
    }

    // Do a change of basis to convert into contact coordinates.
    Matrix3 deltaVelocity = ContactToWorld.transpose();
    deltaVelocity *= deltaVelWorld;
    deltaVelocity *= ContactToWorld;

    // Add in the linear velocity change
    deltaVelocity.data[0] += inverseMass;
    deltaVelocity.data[4] += inverseMass;
    deltaVelocity.data[8] += inverseMass;

    // Invert to get the impulse needed per unit velocity
    Matrix3 impulseMatrix = deltaVelocity.inverse();

    // Find the target velocities to kill
    Vector3 velKill = 
		{ DesiredDeltaVelocity
		, -ContactVelocity.y
		, -ContactVelocity.z
		};

    // Find the impulse to kill target velocities
	Vector3							impulseContact		= impulseMatrix.transform(velKill);

    // Check for exceeding friction
    double planarImpulse = real_sqrt(
        impulseContact.y*impulseContact.y +
        impulseContact.z*impulseContact.z
        );
    if (planarImpulse > impulseContact.x * Friction)
    {
        // We need to use dynamic friction
        impulseContact.y /= planarImpulse;
        impulseContact.z /= planarImpulse;

        impulseContact.x = deltaVelocity.data[0] +
            deltaVelocity.data[1] * Friction * impulseContact.y +
            deltaVelocity.data[2] * Friction * impulseContact.z;
        impulseContact.x = DesiredDeltaVelocity / impulseContact.x;
        impulseContact.y *= Friction * impulseContact.x;
        impulseContact.z *= Friction * impulseContact.x;
    }
    return impulseContact;
}

void Contact::applyPositionChange	( Vector3 linearChange	[2]
									, Vector3 angularChange	[2]
									, double penetration
									)
{
	const double					angularLimit					= (double)0.2f;
	double							angularMove		[2]				= {};
	double							linearMove		[2]				= {};
	
	double							totalInertia					= 0;
	double							linearInertia	[2]				= {};
	double							angularInertia	[2]				= {};
	
	for (uint32_t i = 0; i < 2; i++)	// We need to work out the inertia of each object in the direction of the contact normal, due to angular inertia only.
		if (Body[i]) {
			Matrix3							inverseInertiaTensor = Body[i]->InverseInertiaTensorWorld;

			// Use the same procedure as for calculating frictionless velocity change to work out the angular inertia.
			Vector3							angularInertiaWorld										= RelativeContactPosition[i] % ContactNormal;
			angularInertiaWorld			= inverseInertiaTensor.transform(angularInertiaWorld);
			angularInertiaWorld			= angularInertiaWorld % RelativeContactPosition[i];
			angularInertia	[i]			= angularInertiaWorld * ContactNormal;
			linearInertia	[i]			= Body[i]->InverseMass;	// The linear component is simply the inverse mass
			totalInertia				+= linearInertia[i] + angularInertia[i];	// Keep track of the total inertia from all components

			// We break the loop here so that the totalInertia value is completely calculated (by both iterations) before continuing.
		}

    // Loop through again calculating and applying the changes
    for (uint32_t i = 0; i < 2; i++) 
		if (Body[i]) {
			// The linear and angular movements required are in proportion to the two inverse inertias.
			double sign = (i == 0)?1:-1;
			angularMove[i]	= sign * penetration * (angularInertia	[i] / totalInertia);
			linearMove[i]	= sign * penetration * (linearInertia	[i] / totalInertia);

			// To avoid angular projections that are too great (when mass is large but inertia tensor is small) limit the angular move.
			Vector3 projection = RelativeContactPosition[i];
			projection.addScaledVector(
				ContactNormal,
				-RelativeContactPosition[i].scalarProduct(ContactNormal)
				);

			// Use the small angle approximation for the sine of the angle (i.e. the magnitude would be sine(angularLimit) * projection.magnitude but we approximate sine(angularLimit) to angularLimit).
			double maxMagnitude = angularLimit * projection.magnitude();

			if (angularMove[i] < -maxMagnitude)
			{
				double totalMove = angularMove[i] + linearMove[i];
				angularMove[i] = -maxMagnitude;
				linearMove[i] = totalMove - angularMove[i];
			}
			else if (angularMove[i] > maxMagnitude) {
				double totalMove = angularMove[i] + linearMove[i];
				angularMove[i] = maxMagnitude;
				linearMove[i] = totalMove - angularMove[i];
			}

			// We have the linear amount of movement required by turning the rigid body (in angularMove[i]). We now need to calculate the desired rotation to achieve that.
			if (angularMove[i] == 0)  // Easy case - no angular movement means no rotation.		
				angularChange[i].clear();
			else { // Work out the direction we'd like to rotate in.
				Vector3		targetAngularDirection	= RelativeContactPosition[i].vectorProduct(ContactNormal);
				Matrix3		inverseInertiaTensor	= Body[i]->InverseInertiaTensorWorld;

			    // Work out the direction we'd need to rotate to achieve that
			    angularChange[i] = inverseInertiaTensor.transform(targetAngularDirection) * (angularMove[i] / angularInertia[i]);
			}

			linearChange[i] = ContactNormal * linearMove[i];	// Velocity change is easier - it is just the linear movement along the contact normal.

			// Now we can start to apply the values we've calculated. Apply the linear movement
			Vector3 pos = Body[i]->Position;
			pos.addScaledVector(ContactNormal, linearMove[i]);
			Body[i]->Position = pos;

			// And the change in orientation
			Quaternion q = Body[i]->Orientation;
			q.addScaledVector(angularChange[i], 1.0);
			Body[i]->Orientation = q;

			// We need to calculate the derived data for any body that is asleep, so that the changes are reflected in the object's data. 
			// Otherwise the resolution will not change the position of the object, and the next collision detection round will have the same penetration.
			if (!Body[i]->IsAwake) 
				Body[i]->CalculateDerivedData();
		}
}

// Contact resolver implementation
void ContactResolver::resolveContacts(Contact *contacts,
                                      uint32_t numContacts,
                                      double duration)
{
    // Make sure we have something to do.
    if (numContacts == 0) 
		return;
    if (!isValid()) 
		return;
    prepareContacts(contacts, numContacts, duration);	// Prepare the contacts for processing
    adjustPositions(contacts, numContacts, duration);	// Resolve the interpenetration problems with the contacts.
    adjustVelocities(contacts, numContacts, duration);	// Resolve the velocity problems with the contacts.
}

void ContactResolver::prepareContacts(Contact* contacts,
                                      uint32_t numContacts,
                                      double duration)
{
    // Generate contact velocity and axis information.
    Contact* lastContact = contacts + numContacts;
    for (Contact* contact=contacts; contact < lastContact; contact++)
        contact->calculateInternals(duration);	// Calculate the internal contact data (inertia, basis, etc).
}

void ContactResolver::adjustVelocities(Contact *c,
                                       uint32_t numContacts,
                                       double duration)
{
    Vector3 velocityChange[2], rotationChange[2];
    Vector3 deltaVel;

	// iteratively handle impacts in order of severity.
	VelocityIterationsUsed = 0;
	while (VelocityIterationsUsed < VelocityIterations) {
		// Find contact with maximum magnitude of probable velocity change.
		double max = VelocityEpsilon;
		uint32_t index = numContacts;
		for (uint32_t i = 0; i < numContacts; i++)
		{
		    if (c[i].DesiredDeltaVelocity > max)
		    {
		        max = c[i].DesiredDeltaVelocity;
		        index = i;
		    }
		}
		if (index == numContacts) 
			break;

		c[index].matchAwakeState();	// Match the awake state at the contact
		c[index].applyVelocityChange(velocityChange, rotationChange);	// Do the resolution on the contact that came out top.

		
		for (uint32_t i = 0; i < numContacts; i++) // With the change in velocity of the two bodies, the update of contact velocities means that some of the relative closing velocities need recomputing.
			for (uint32_t b = 0; b < 2; b++) if (c[i].Body[b])	// Check each body in the contact
				for (uint32_t d = 0; d < 2; d++) // Check for a match with each body in the newly resolved contact
					if (c[i].Body[b] == c[index].Body[d]) {
						deltaVel						= velocityChange[d] + rotationChange[d].vectorProduct(c[i].RelativeContactPosition[b]);
						c[i].ContactVelocity			+= c[i].ContactToWorld.transformTranspose(deltaVel) * ( b ? -1 : 1);	// The sign of the change is negative if we're dealing with the second body in a contact.
						c[i].calculateDesiredDeltaVelocity(duration);
					}
		VelocityIterationsUsed++;
	}
}

void ContactResolver::adjustPositions(Contact *c,
                                      uint32_t numContacts,
                                      double duration)
{
    uint32_t	i, index;
    Vector3		linearChange[2], angularChange[2];
    double		max;
    Vector3		deltaPosition;

    // iteratively resolve interpenetrations in order of severity.
    PositionIterationsUsed = 0;
    while (PositionIterationsUsed < PositionIterations)
    {
        // Find biggest penetration
        max = PositionEpsilon;
        index = numContacts;
        for (i=0; i<numContacts; i++)
        {
            if (c[i].Penetration > max)
            {
                max = c[i].Penetration;
                index = i;
            }
        }
        if (index == numContacts) 
			break;

        // Match the awake state at the contact
        c[index].matchAwakeState();

        // Resolve the penetration.
        c[index].applyPositionChange(
            linearChange,
            angularChange,
            max);

        // Again this action may have changed the penetration of other bodies, so we update contacts.
        for (i = 0; i < numContacts; i++)
        {
            // Check each body in the contact
            for (unsigned b = 0; b < 2; b++) if (c[i].Body[b])
            {
                // Check for a match with each body in the newly resolved contact
                for (unsigned d = 0; d < 2; d++)
                {
                    if (c[i].Body[b] == c[index].Body[d])
                    {
                        deltaPosition = linearChange[d] +
                            angularChange[d].vectorProduct(
                                c[i].RelativeContactPosition[b]);

                        // The sign of the change is positive if we're dealing with the second body in a contact and negative otherwise (because we're subtracting the resolution).
                        c[i].Penetration +=
                            deltaPosition.scalarProduct(c[i].ContactNormal)
                            * (b?1:-1);
                    }
                }
            }
        }
        PositionIterationsUsed++;
    }
}
