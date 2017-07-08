// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include "body.h"

#include <memory.h>
#include <assert.h>

using namespace cyclone;
// Internal function that checks the validity of an inverse inertia tensor.
static inline constexpr	void			_checkInverseInertiaTensor				(const Matrix3 &/*iitWorld*/)		{}	// TODO: Perform a validity check in an assert.

// Internal function to do an intertia tensor transform by a quaternion.
// Note that the implementation of this function was created by an automated code-generator and optimizer.
static inline void _transformInertiaTensor
	(	Matrix3				& iitWorld
	,	const Quaternion	& //q
	,	const Matrix3		& iitBody
	,	const Matrix4		& rotmat
	)
{
    real t4 = rotmat.data[0]*iitBody.data[0]+
        rotmat.data[1]*iitBody.data[3]+
        rotmat.data[2]*iitBody.data[6];
    real t9 = rotmat.data[0]*iitBody.data[1]+
        rotmat.data[1]*iitBody.data[4]+
        rotmat.data[2]*iitBody.data[7];
    real t14 = rotmat.data[0]*iitBody.data[2]+
        rotmat.data[1]*iitBody.data[5]+
        rotmat.data[2]*iitBody.data[8];
    real t28 = rotmat.data[4]*iitBody.data[0]+
        rotmat.data[5]*iitBody.data[3]+
        rotmat.data[6]*iitBody.data[6];
    real t33 = rotmat.data[4]*iitBody.data[1]+
        rotmat.data[5]*iitBody.data[4]+
        rotmat.data[6]*iitBody.data[7];
    real t38 = rotmat.data[4]*iitBody.data[2]+
        rotmat.data[5]*iitBody.data[5]+
        rotmat.data[6]*iitBody.data[8];
    real t52 = rotmat.data[8]*iitBody.data[0]+
        rotmat.data[9]*iitBody.data[3]+
        rotmat.data[10]*iitBody.data[6];
    real t57 = rotmat.data[8]*iitBody.data[1]+
        rotmat.data[9]*iitBody.data[4]+
        rotmat.data[10]*iitBody.data[7];
    real t62 = rotmat.data[8]*iitBody.data[2]+
        rotmat.data[9]*iitBody.data[5]+
        rotmat.data[10]*iitBody.data[8];

    iitWorld.data[0] = t4*rotmat.data[0]+
        t9*rotmat.data[1]+
        t14*rotmat.data[2];
    iitWorld.data[1] = t4*rotmat.data[4]+
        t9*rotmat.data[5]+
        t14*rotmat.data[6];
    iitWorld.data[2] = t4*rotmat.data[8]+
        t9*rotmat.data[9]+
        t14*rotmat.data[10];
    iitWorld.data[3] = t28*rotmat.data[0]+
        t33*rotmat.data[1]+
        t38*rotmat.data[2];
    iitWorld.data[4] = t28*rotmat.data[4]+
        t33*rotmat.data[5]+
        t38*rotmat.data[6];
    iitWorld.data[5] = t28*rotmat.data[8]+
        t33*rotmat.data[9]+
        t38*rotmat.data[10];
    iitWorld.data[6] = t52*rotmat.data[0]+
        t57*rotmat.data[1]+
        t62*rotmat.data[2];
    iitWorld.data[7] = t52*rotmat.data[4]+
        t57*rotmat.data[5]+
        t62*rotmat.data[6];
    iitWorld.data[8] = t52*rotmat.data[8]+
        t57*rotmat.data[9]+
        t62*rotmat.data[10];
}

/**
 * Inline function that creates a transform matrix from a
 * position and orientation.
 */
static inline void _calculateTransformMatrix(Matrix4 &transformMatrix,
                                             const Vector3 &position,
                                             const Quaternion &orientation)
{
    transformMatrix.data[0] = 1-2*orientation.j*orientation.j-
        2*orientation.k*orientation.k;
    transformMatrix.data[1] = 2*orientation.i*orientation.j -
        2*orientation.r*orientation.k;
    transformMatrix.data[2] = 2*orientation.i*orientation.k +
        2*orientation.r*orientation.j;
    transformMatrix.data[3] = position.x;

    transformMatrix.data[4] = 2*orientation.i*orientation.j +
        2*orientation.r*orientation.k;
    transformMatrix.data[5] = 1-2*orientation.i*orientation.i-
        2*orientation.k*orientation.k;
    transformMatrix.data[6] = 2*orientation.j*orientation.k -
        2*orientation.r*orientation.i;
    transformMatrix.data[7] = position.y;

    transformMatrix.data[8] = 2*orientation.i*orientation.k -
        2*orientation.r*orientation.j;
    transformMatrix.data[9] = 2*orientation.j*orientation.k +
        2*orientation.r*orientation.i;
    transformMatrix.data[10] = 1-2*orientation.i*orientation.i-
        2*orientation.j*orientation.j;
    transformMatrix.data[11] = position.z;
}

/*
 * --------------------------------------------------------------------------
 * FUNCTIONS DECLARED IN HEADER:
 * --------------------------------------------------------------------------
 */
void RigidBody::calculateDerivedData() {
	Orientation.normalise();
	_calculateTransformMatrix	(TransformMatrix, Position, Orientation);	// Calculate the transform matrix for the body.
	_transformInertiaTensor		(InverseInertiaTensorWorld, Orientation, InverseInertiaTensor, TransformMatrix);	// Calculate the inertiaTensor in world space.
}

void RigidBody::integrate(real duration){
	if (!IsAwake) 
		return;
	
	// Calculate linear acceleration from force inputs.
	LastFrameAcceleration = Acceleration;
	LastFrameAcceleration.addScaledVector(AccumulatedForce, InverseMass);
	
	// Calculate angular acceleration from torque inputs.
	Vector3 angularAcceleration = InverseInertiaTensorWorld.transform(TorqueAccum);
	
	// Adjust velocities
	Velocity.addScaledVector(LastFrameAcceleration, duration);	// Update linear velocity from both acceleration and impulse.
	Rotation.addScaledVector(angularAcceleration, duration);	// Update angular velocity from both acceleration and impulse.
	
	// Impose drag.
	Velocity *= real_pow(LinearDamping, duration);
	Rotation *= real_pow(AngularDamping, duration);
	
	// Adjust positions
	Position	.addScaledVector(Velocity, duration);	// Update linear position.
	Orientation	.addScaledVector(Rotation, duration);	// Update angular position.
	calculateDerivedData();			// Normalise the orientation, and update the matrices with the new position and orientation
	clearAccumulators();			// Clear accumulators.
	
	if (CanSleep) {	// Update the kinetic energy store, and possibly put the body to sleep.
		real							currentMotion	= Velocity.scalarProduct(Velocity) + Rotation.scalarProduct(Rotation);
		real							bias			= real_pow(0.5, duration);
		Motion						= bias * Motion + (1 - bias) * currentMotion;
		if (Motion < sleepEpsilon) 
			setAwake(false);
		else if (Motion > 10 * sleepEpsilon) 
			Motion		= 10 * sleepEpsilon;
		}
}

void RigidBody::setMass(const real mass)
{
    assert(mass != 0);
    RigidBody::InverseMass = ((real)1.0)/mass;
}

real RigidBody::getMass() const
{
    if (InverseMass == 0) {
        return REAL_MAX;
    } else {
        return ((real)1.0)/InverseMass;
    }
}

bool RigidBody::hasFiniteMass() const
{
    return InverseMass >= 0.0f;
}

void RigidBody::setInertiaTensor(const Matrix3 &inertiaTensor)
{
    InverseInertiaTensor.setInverse(inertiaTensor);
    _checkInverseInertiaTensor(InverseInertiaTensor);
}

void RigidBody::getInertiaTensor(Matrix3 *inertiaTensor) const
{
    inertiaTensor->setInverse(InverseInertiaTensor);
}

Matrix3 RigidBody::getInertiaTensor() const
{
    Matrix3 it;
    getInertiaTensor(&it);
    return it;
}

void RigidBody::getInertiaTensorWorld(Matrix3 *inertiaTensor) const
{
    inertiaTensor->setInverse(InverseInertiaTensorWorld);
}

Matrix3 RigidBody::getInertiaTensorWorld() const
{
    Matrix3 it;
    getInertiaTensorWorld(&it);
    return it;
}

void RigidBody::setInverseInertiaTensor(const Matrix3 &inverseInertiaTensor)
{
	_checkInverseInertiaTensor(inverseInertiaTensor);
	RigidBody::InverseInertiaTensor = inverseInertiaTensor;
}

void RigidBody::setDamping(const real linearDamping,
               const real angularDamping)
{
    RigidBody::LinearDamping = linearDamping;
    RigidBody::AngularDamping = angularDamping;
}



void RigidBody::getPosition(Vector3 *position) const
{
    *position = RigidBody::Position;
}

void RigidBody::setOrientation(const Quaternion &orientation)
{
    RigidBody::Orientation = orientation;
    RigidBody::Orientation.normalise();
}

void RigidBody::setOrientation(const real r, const real i,
                   const real j, const real k)
{
    Orientation.r = r;
    Orientation.i = i;
    Orientation.j = j;
    Orientation.k = k;
    Orientation.normalise();
}

void RigidBody::getOrientation(Matrix3 *matrix) const
{
    getOrientation(matrix->data);
}

void RigidBody::getOrientation(real matrix[9]) const
{
    matrix[0] = TransformMatrix.data[0];
    matrix[1] = TransformMatrix.data[1];
    matrix[2] = TransformMatrix.data[2];

    matrix[3] = TransformMatrix.data[4];
    matrix[4] = TransformMatrix.data[5];
    matrix[5] = TransformMatrix.data[6];

    matrix[6] = TransformMatrix.data[8];
    matrix[7] = TransformMatrix.data[9];
    matrix[8] = TransformMatrix.data[10];
}

void RigidBody::getTransform(Matrix4 *transform) const
{
    memcpy(transform, &TransformMatrix.data, sizeof(Matrix4));
}

void RigidBody::getTransform(real matrix[16]) const
{
    memcpy(matrix, TransformMatrix.data, sizeof(real)*12);
    matrix[12] = matrix[13] = matrix[14] = 0;
    matrix[15] = 1;
}

void RigidBody::getGLTransform(float matrix[16]) const
{
    matrix[0] = (float)TransformMatrix.data[0];
    matrix[1] = (float)TransformMatrix.data[4];
    matrix[2] = (float)TransformMatrix.data[8];
    matrix[3] = 0;

    matrix[4] = (float)TransformMatrix.data[1];
    matrix[5] = (float)TransformMatrix.data[5];
    matrix[6] = (float)TransformMatrix.data[9];
    matrix[7] = 0;

    matrix[8] = (float)TransformMatrix.data[2];
    matrix[9] = (float)TransformMatrix.data[6];
    matrix[10] = (float)TransformMatrix.data[10];
    matrix[11] = 0;

    matrix[12] = (float)TransformMatrix.data[3];
    matrix[13] = (float)TransformMatrix.data[7];
    matrix[14] = (float)TransformMatrix.data[11];
    matrix[15] = 1;
}

Matrix4 RigidBody::getTransform() const
{
    return TransformMatrix;
}


Vector3 RigidBody::GetPointInLocalSpace(const Vector3 &point) const
{
    return TransformMatrix.transformInverse(point);
}

Vector3 RigidBody::getPointInWorldSpace(const Vector3 &point) const
{
    return TransformMatrix.transform(point);
}

Vector3 RigidBody::getDirectionInLocalSpace(const Vector3 &direction) const
{
    return TransformMatrix.transformInverseDirection(direction);
}

Vector3 RigidBody::getDirectionInWorldSpace(const Vector3 &direction) const
{
    return TransformMatrix.transformDirection(direction);
}

void RigidBody::setAwake(const bool awake)
{
    if (awake) {
        IsAwake = true;
        Motion = sleepEpsilon * 2.0f;	// Add a bit of motion to avoid it falling asleep immediately.
    } else {
        IsAwake = false;
        Velocity.clear();
        Rotation.clear();
    }
}

void RigidBody::setCanSleep(const bool canSleep) {
	RigidBody::CanSleep = canSleep;
	if (!canSleep && !IsAwake) 
		setAwake();
}

void RigidBody::clearAccumulators()
{
    AccumulatedForce.clear();
    TorqueAccum.clear();
}

void RigidBody::addForce(const Vector3 &force)
{
    AccumulatedForce += force;
    IsAwake = true;
}

void RigidBody::addForceAtBodyPoint(const Vector3 &force,
                                    const Vector3 &point)
{
    // Convert to coordinates relative to center of mass.
    Vector3 pt = getPointInWorldSpace(point);
    addForceAtPoint(force, pt);

}

void RigidBody::addForceAtPoint(const Vector3 &force,
                                const Vector3 &point)
{
    // Convert to coordinates relative to center of mass.
    Vector3 pt = point;
    pt -= Position;

    AccumulatedForce += force;
    TorqueAccum += pt % force;

    IsAwake = true;
}

void RigidBody::addTorque(const Vector3 &torque)
{
    TorqueAccum += torque;
    IsAwake = true;
}
