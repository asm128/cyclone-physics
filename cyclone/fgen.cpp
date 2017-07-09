// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include "fgen.h"

using namespace cyclone;

void ForceRegistry::UpdateForces(double duration) {
	Registry::iterator i = registrations.begin();
	for (; i != registrations.end(); i++)
		i->fg->UpdateForce(i->body, duration);
}

void ForceRegistry::add(RigidBody *body, ForceGenerator *fg)
{
    ForceRegistry::ForceRegistration registration;
    registration.body = body;
    registration.fg = fg;
    registrations.push_back(registration);
}

Buoyancy::Buoyancy(const Vector3 &cOfB, double maxDepth, double volume, double waterHeight, double liquidDensity /* = 1000.0f */)
{
    centreOfBuoyancy = cOfB;
    Buoyancy::liquidDensity = liquidDensity;
    Buoyancy::maxDepth = maxDepth;
    Buoyancy::volume = volume;
    Buoyancy::waterHeight = waterHeight;
}

void Buoyancy::UpdateForce(RigidBody *body, double duration)
{
    // Calculate the submersion depth
    Vector3		pointInWorld = body->getPointInWorldSpace(centreOfBuoyancy);
    double		depth = pointInWorld.y;

    // Check if we're out of the water
    if (depth >= waterHeight + maxDepth) 
		return;
	Vector3		force = {};

    // Check if we're at maximum depth
    if (depth <= waterHeight - maxDepth) {
        force.y = liquidDensity * volume;
        body->addForceAtBodyPoint(force, centreOfBuoyancy);
        return;
    }

    // Otherwise we are partly submerged
    force.y = liquidDensity * volume *
        (depth - maxDepth - waterHeight) / 2 * maxDepth;
    body->addForceAtBodyPoint(force, centreOfBuoyancy);
}

Spring::Spring(const Vector3 &localConnectionPt,
               RigidBody *other,
               const Vector3 &otherConnectionPt,
               double springConstant,
               double restLength)
: connectionPoint(localConnectionPt),
  otherConnectionPoint(otherConnectionPt),
  other(other),
  springConstant(springConstant),
  restLength(restLength)
{
}

void Spring::UpdateForce(RigidBody* body, double duration)
{
    // Calculate the two ends in world space
    Vector3 lws = body->getPointInWorldSpace(connectionPoint);
    Vector3 ows = other->getPointInWorldSpace(otherConnectionPoint);

    // Calculate the vector of the spring
    Vector3 force = lws - ows;

    // Calculate the magnitude of the force
    double magnitude = force.magnitude();
    magnitude = real_abs(magnitude - restLength);
    magnitude *= springConstant;

    // Calculate the final force and apply it
    force.normalise();
    force *= -magnitude;
    body->addForceAtPoint(force, lws);
}

void Aero::UpdateForce				(RigidBody *body, double duration)							{ Aero::UpdateForceFromTensor(body, duration, Tensor); }
void Aero::UpdateForceFromTensor	(RigidBody *body, double duration, const Matrix3 &tensor)	{
    // Calculate total velocity (windspeed and body's velocity).
    Vector3 velocity = body->Velocity;
    velocity += *Windspeed;

    Vector3 bodyVel = body->getTransform().transformInverseDirection(velocity);		// Calculate the velocity in body coordinates

    // Calculate the force in body coordinates
    Vector3 bodyForce = tensor.transform(bodyVel);
    Vector3 force = body->getTransform().transformDirection(bodyForce);

    body->addForceAtBodyPoint(force, Position);		// Apply the force
}

AeroControl::AeroControl(const Matrix3 &base, const Matrix3 &min, const Matrix3 &max,
                              const Vector3 &position, const Vector3 *windspeed)
: Aero(base, position, windspeed)
{
    MinTensor		= min;
    MaxTensor		= max;
    ControlSetting	= 0.0f;
}

Matrix3 AeroControl::getTensor() {
		 if (ControlSetting <= -1.0f)	return MinTensor;
    else if (ControlSetting >= 1.0f)	return MaxTensor;
    else if (ControlSetting < 0)		return Matrix3::linearInterpolate(MinTensor, Tensor, ControlSetting + 1.0f);
    else if (ControlSetting > 0)		return Matrix3::linearInterpolate(Tensor, MaxTensor, ControlSetting);
    else 
		return Tensor;
}

