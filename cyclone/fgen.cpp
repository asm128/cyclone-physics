// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include "fgen.h"

using namespace cyclone;

void ForceRegistry::UpdateForces(real duration)
{
    Registry::iterator i = registrations.begin();
    for (; i != registrations.end(); i++)
    {
        i->fg->UpdateForce(i->body, duration);
    }
}

void ForceRegistry::add(RigidBody *body, ForceGenerator *fg)
{
    ForceRegistry::ForceRegistration registration;
    registration.body = body;
    registration.fg = fg;
    registrations.push_back(registration);
}

Buoyancy::Buoyancy(const Vector3 &cOfB, real maxDepth, real volume,
                   real waterHeight, real liquidDensity /* = 1000.0f */)
{
    centreOfBuoyancy = cOfB;
    Buoyancy::liquidDensity = liquidDensity;
    Buoyancy::maxDepth = maxDepth;
    Buoyancy::volume = volume;
    Buoyancy::waterHeight = waterHeight;
}

void Buoyancy::UpdateForce(RigidBody *body, real duration)
{
    // Calculate the submersion depth
    Vector3 pointInWorld = body->getPointInWorldSpace(centreOfBuoyancy);
    real depth = pointInWorld.y;

    // Check if we're out of the water
    if (depth >= waterHeight + maxDepth) return;
	Vector3 force = {};

    // Check if we're at maximum depth
    if (depth <= waterHeight - maxDepth)
    {
        force.y = liquidDensity * volume;
        body->addForceAtBodyPoint(force, centreOfBuoyancy);
        return;
    }

    // Otherwise we are partly submerged
    force.y = liquidDensity * volume *
        (depth - maxDepth - waterHeight) / 2 * maxDepth;
    body->addForceAtBodyPoint(force, centreOfBuoyancy);
}

Gravity::Gravity(const Vector3& gravity)
: gravity(gravity)
{
}

void Gravity::UpdateForce(RigidBody* body, real duration)
{
    // Check that we do not have infinite mass
    if (!body->hasFiniteMass()) return;

    // Apply the mass-scaled force to the body
    body->addForce(gravity * body->getMass());
}

Spring::Spring(const Vector3 &localConnectionPt,
               RigidBody *other,
               const Vector3 &otherConnectionPt,
               real springConstant,
               real restLength)
: connectionPoint(localConnectionPt),
  otherConnectionPoint(otherConnectionPt),
  other(other),
  springConstant(springConstant),
  restLength(restLength)
{
}

void Spring::UpdateForce(RigidBody* body, real duration)
{
    // Calculate the two ends in world space
    Vector3 lws = body->getPointInWorldSpace(connectionPoint);
    Vector3 ows = other->getPointInWorldSpace(otherConnectionPoint);

    // Calculate the vector of the spring
    Vector3 force = lws - ows;

    // Calculate the magnitude of the force
    real magnitude = force.magnitude();
    magnitude = real_abs(magnitude - restLength);
    magnitude *= springConstant;

    // Calculate the final force and apply it
    force.normalise();
    force *= -magnitude;
    body->addForceAtPoint(force, lws);
}

void Aero::UpdateForce(RigidBody *body, real duration)
{
    Aero::UpdateForceFromTensor(body, duration, Tensor);
}

void Aero::UpdateForceFromTensor(RigidBody *body, real duration,
                                 const Matrix3 &tensor)
{
    // Calculate total velocity (windspeed and body's velocity).
    Vector3 velocity = body->Velocity;
    velocity += *Windspeed;

    // Calculate the velocity in body coordinates
    Vector3 bodyVel = body->getTransform().transformInverseDirection(velocity);

    // Calculate the force in body coordinates
    Vector3 bodyForce = tensor.transform(bodyVel);
    Vector3 force = body->getTransform().transformDirection(bodyForce);

    // Apply the force
    body->addForceAtBodyPoint(force, Position);
}

AeroControl::AeroControl(const Matrix3 &base, const Matrix3 &min, const Matrix3 &max,
                              const Vector3 &position, const Vector3 *windspeed)
:
Aero(base, position, windspeed)
{
    AeroControl::minTensor = min;
    AeroControl::maxTensor = max;
    controlSetting = 0.0f;
}

Matrix3 AeroControl::getTensor() {
		 if (controlSetting <= -1.0f)	return minTensor;
    else if (controlSetting >= 1.0f)	return maxTensor;
    else if (controlSetting < 0)		return Matrix3::linearInterpolate(minTensor, Tensor, controlSetting + 1.0f);
    else if (controlSetting > 0)		return Matrix3::linearInterpolate(Tensor, maxTensor, controlSetting);
    else 
		return Tensor;
}

void AeroControl::setControl(real value)
{
    controlSetting = value;
}

void AeroControl::UpdateForce(RigidBody *body, real duration)
{
    Matrix3 tensor = getTensor();
    Aero::UpdateForceFromTensor(body, duration, tensor);
}
