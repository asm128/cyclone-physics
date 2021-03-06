// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include "fgen.h"

using namespace cyclone;

								Buoyancy::Buoyancy				(const Vector3 &centreOfBuoyancy, double maxDepth, double volume, double waterHeight, double liquidDensity /* = 1000.0f */)		{
	CentreOfBuoyancy	= centreOfBuoyancy;
	LiquidDensity		= liquidDensity;
	MaxDepth			= maxDepth;
	Volume				= volume;
	WaterHeight			= waterHeight;
}

void							Buoyancy::UpdateForce			(RigidBody *body, double duration)																								{
	// Calculate the submersion depth
	Vector3								pointInWorld					= body->getPointInWorldSpace(CentreOfBuoyancy);
	double								depth							= pointInWorld.y;

	if (depth >= WaterHeight + MaxDepth)	// Check if we're out of the water
		return;

	Vector3								force							= {};
	if (depth <= WaterHeight - MaxDepth) {	// Check if we're at maximum depth
		force.y							= LiquidDensity * Volume;
		body->addForceAtBodyPoint(force, CentreOfBuoyancy);
		return;
	}

	// Otherwise we are partly submerged
	force.y							= LiquidDensity * Volume * (depth - MaxDepth - WaterHeight) / 2 * MaxDepth;
	body->addForceAtBodyPoint(force, CentreOfBuoyancy);
}

								Spring::Spring
	( const Vector3		& localConnectionPt
	, RigidBody			* other
	, const Vector3		& otherConnectionPt
	, double			springConstant
	, double			restLength
	)
	: ConnectionPoint		(localConnectionPt)
	, OtherConnectionPoint	(otherConnectionPt)
	, Other					(other)
	, SpringConstant		(springConstant)
	, RestLength			(restLength)
{}

void							Spring::UpdateForce				(RigidBody* body, double duration)																								{
	// Calculate the two ends in world space
	Vector3								lws								= body->getPointInWorldSpace(ConnectionPoint);
	Vector3								ows								= Other->getPointInWorldSpace(OtherConnectionPoint);
	Vector3								force							= lws - ows;	// Calculate the vector of the spring

	// Calculate the magnitude of the force
	double								magnitude						= force.magnitude();
	magnitude						= real_abs(magnitude - RestLength);
	magnitude						*= SpringConstant;

	// Calculate the final force and apply it
	force.normalise();
	force							*= -magnitude;
	body->addForceAtPoint(force, lws);
}

void							 Aero::UpdateForceFromTensor	(RigidBody *body, double duration, const Matrix3 &tensor)																		{
	// Calculate total velocity (windspeed and body's velocity).
	Vector3								velocity						= body->Force.Velocity;
	velocity						+= *Windspeed;

	Vector3								bodyVel							= body->TransformMatrix.transformInverseDirection(velocity);		// Calculate the velocity in body coordinates

	// Calculate the force in body coordinates
	Vector3								bodyForce						= tensor.transform(bodyVel);
	Vector3								force							= body->TransformMatrix.transformDirection(bodyForce);

	body->addForceAtBodyPoint(force, Position);		// Apply the force
}

Matrix3							AeroControl::GetTensor			()																																{
		 if (ControlSetting <= -1.0f)	return MinTensor;
	else if (ControlSetting >= 1.0f)	return MaxTensor;
	else if (ControlSetting < 0)		return Matrix3::linearInterpolate(MinTensor, Tensor, ControlSetting + 1.0f);
	else if (ControlSetting > 0)		return Matrix3::linearInterpolate(Tensor, MaxTensor, ControlSetting);
	else 
		return Tensor;
}
